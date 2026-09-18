#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

/*
 * offboard_controller.cpp - C++ fast PX4 offboard for NIDAR
 * 20Hz control loop, ENU->NED conversion, state machine
 * Much faster than Python: no GIL, direct memory
 */

using namespace std::chrono_literals;

class OffboardController : public rclcpp::Node {
public:
  OffboardController() : Node("offboard_controller") {
    this->declare_parameter("takeoff_height", -1.5);
    takeoff_h_ = this->get_parameter("takeoff_height").as_double();

    rclcpp::QoS qos(1);
    qos.best_effort();

    offboard_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/offboard/pose", 10);

    local_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos, std::bind(&OffboardController::localCb, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos, std::bind(&OffboardController::statusCb, this, std::placeholders::_1));
    traj_sub_ = this->create_subscription<nav_msgs::msg::Path>("/trajectory", 10, std::bind(&OffboardController::trajCb, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/exploration/goal", 10, std::bind(&OffboardController::goalCb, this, std::placeholders::_1));

    state_="IDLE";
    timer_ = this->create_wall_timer(50ms, std::bind(&OffboardController::timerCb, this));
    RCLCPP_INFO(this->get_logger(), "C++ Offboard Controller - takeoff %.2f NED", takeoff_h_);
  }

private:
  void localCb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    local_=*msg; has_local_=true;
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp=this->now(); p.header.frame_id="map";
    p.pose.position.x=msg->y; p.pose.position.y=msg->x; p.pose.position.z=-msg->z;
    p.pose.orientation.w=1.0;
    pose_pub_->publish(p);
  }
  void statusCb(const px4_msgs::msg::VehicleStatus::SharedPtr msg) { status_=*msg; }
  void trajCb(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) return;
    traj_=*msg; idx_=0;
    RCLCPP_INFO(this->get_logger(), "Trajectory %zu waypoints", msg->poses.size());
    if (state_=="TAKEOFF") state_="EXPLORING";
  }
  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (traj_.poses.empty() || idx_>=traj_.poses.size()) {
      nav_msgs::msg::Path path; path.header=msg->header; path.poses.push_back(*msg);
      traj_=path; idx_=0;
    }
  }

  void publishOffboardMode() {
    px4_msgs::msg::OffboardControlMode m;
    m.position=true; m.velocity=false; m.acceleration=false; m.attitude=false; m.body_rate=false;
    m.timestamp=this->get_clock()->now().nanoseconds()/1000;
    offboard_pub_->publish(m);
  }
  void publishSetpoint(double x_ned, double y_ned, double z_ned, double yaw=0) {
    px4_msgs::msg::TrajectorySetpoint s;
    s.position={static_cast<float>(x_ned), static_cast<float>(y_ned), static_cast<float>(z_ned)};
    s.yaw=yaw;
    s.timestamp=this->get_clock()->now().nanoseconds()/1000;
    traj_pub_->publish(s);
  }
  void sendCmd(int cmd, float p1=0, float p2=0) {
    px4_msgs::msg::VehicleCommand c;
    c.command=cmd; c.param1=p1; c.param2=p2;
    c.target_system=1; c.target_component=1; c.source_system=1; c.source_component=1; c.from_external=true;
    c.timestamp=this->get_clock()->now().nanoseconds()/1000;
    cmd_pub_->publish(c);
  }

  void timerCb() {
    publishOffboardMode();
    if (!has_local_) { publishSetpoint(0,0,takeoff_h_); counter_++; return; }

    if (state_=="IDLE") {
      if (counter_<20) { publishSetpoint(0,0,takeoff_h_); counter_++; }
      else { sendCmd(176,1,6); state_="ARMING"; arm_time_=this->now(); RCLCPP_INFO(this->get_logger(), "Offboard mode"); }
    } else if (state_=="ARMING") {
      publishSetpoint(0,0,takeoff_h_);
      if ((this->now()-arm_time_).seconds()>1.0) { sendCmd(400,1,0); state_="TAKEOFF"; takeoff_start_=this->now(); RCLCPP_INFO(this->get_logger(), "Arming -> TAKEOFF"); }
    } else if (state_=="TAKEOFF") {
      publishSetpoint(0,0,takeoff_h_);
      if (std::abs(local_.z - takeoff_h_)<0.3 && (this->now()-takeoff_start_).seconds()>3.0) {
        RCLCPP_INFO(this->get_logger(), "Takeoff complete");
      }
    } else if (state_=="EXPLORING") {
      if (traj_.poses.empty() || idx_>=traj_.poses.size()) {
        publishSetpoint(local_.x, local_.y, takeoff_h_);
      } else {
        auto &t=traj_.poses[idx_];
        double x_ned=t.pose.position.y;
        double y_ned=t.pose.position.x;
        double z_ned=-t.pose.position.z;
        z_ned=std::max(z_ned, takeoff_h_-0.5); z_ned=std::min(z_ned, -0.5);
        publishSetpoint(x_ned,y_ned,z_ned);
        double dx=local_.x-x_ned, dy=local_.y-y_ned;
        if (std::hypot(dx,dy)<0.4) { idx_++; if (idx_>=traj_.poses.size()) RCLCPP_INFO(this->get_logger(), "Traj complete"); }
      }
    }
  }

  double takeoff_h_;
  std::string state_;
  int counter_=0;
  size_t idx_=0;
  nav_msgs::msg::Path traj_;
  px4_msgs::msg::VehicleLocalPosition local_;
  px4_msgs::msg::VehicleStatus status_;
  bool has_local_=false;
  rclcpp::Time arm_time_, takeoff_start_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr traj_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr traj_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardController>());
  rclcpp::shutdown();
  return 0;
}
