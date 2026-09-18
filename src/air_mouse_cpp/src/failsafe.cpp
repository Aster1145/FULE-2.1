#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <chrono>

class FailsafeNode : public rclcpp::Node {
public:
  FailsafeNode() : Node("failsafe_node") {
    this->declare_parameter("max_height", 2.44);
    this->declare_parameter("arena_size", 15.0);
    this->declare_parameter("vio_timeout", 2.0);
    this->declare_parameter("lidar_timeout", 2.0);
    this->declare_parameter("low_battery", 14.0);

    max_h_=this->get_parameter("max_height").as_double();
    arena_=this->get_parameter("arena_size").as_double();
    vio_timeout_=this->get_parameter("vio_timeout").as_double();
    lidar_timeout_=this->get_parameter("lidar_timeout").as_double();
    low_batt_=this->get_parameter("low_battery").as_double();

    pose_sub_=this->create_subscription<geometry_msgs::msg::PoseStamped>("/offboard/pose", 10, std::bind(&FailsafeNode::poseCb, this, std::placeholders::_1));
    scan_sub_=this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&FailsafeNode::scanCb, this, std::placeholders::_1));
    vio_sub_=this->create_subscription<geometry_msgs::msg::PoseStamped>("/vins_estimator/camera_pose", 10, std::bind(&FailsafeNode::vioCb, this, std::placeholders::_1));
    e_stop_sub_=this->create_subscription<std_msgs::msg::Bool>("/emergency_stop", 10, std::bind(&FailsafeNode::eStopCb, this, std::placeholders::_1));
    batt_sub_=this->create_subscription<px4_msgs::msg::BatteryStatus>("/fmu/out/battery_status", 10, std::bind(&FailsafeNode::battCb, this, std::placeholders::_1));

    abort_pub_=this->create_publisher<std_msgs::msg::Bool>("/mission/abort", 10);
    reason_pub_=this->create_publisher<std_msgs::msg::String>("/failsafe/reason", 10);
    status_pub_=this->create_publisher<std_msgs::msg::String>("/failsafe/status", 10);

    last_vio_=this->now();
    last_lidar_=this->now();

    timer_=this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&FailsafeNode::check, this));
    RCLCPP_INFO(this->get_logger(), "C++ Failsafe max_h %.2f arena %.1f", max_h_, arena_);
  }

private:
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_=*msg; has_pose_=true; }
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr) { last_lidar_=this->now(); }
  void vioCb(const geometry_msgs::msg::PoseStamped::SharedPtr) { last_vio_=this->now(); }
  void eStopCb(const std_msgs::msg::Bool::SharedPtr msg) { if (msg->data) trigger("Emergency stop"); }
  void battCb(const px4_msgs::msg::BatteryStatus::SharedPtr msg) { voltage_=msg->voltage_v; }

  void check() {
    std::string reason;
    if (emergency_) reason="Emergency stop";
    if (has_pose_ && pose_.pose.position.z > max_h_) reason="Height limit "+std::to_string(pose_.pose.position.z)+" > "+std::to_string(max_h_);
    if (has_pose_) {
      double x=pose_.pose.position.x, y=pose_.pose.position.y;
      if (std::abs(x)>arena_/2 || std::abs(y)>arena_/2) reason="Geofence breach";
    }
    if ((this->now()-last_vio_).seconds() > vio_timeout_) reason="VIO timeout";
    if ((this->now()-last_lidar_).seconds() > lidar_timeout_) reason="Lidar timeout";
    if (voltage_>0 && voltage_ < low_batt_) reason="Low battery "+std::to_string(voltage_);

    if (!reason.empty()) trigger(reason);
    else {
      std_msgs::msg::String s; s.data="OK"; status_pub_->publish(s);
    }
  }

  void trigger(const std::string& reason) {
    RCLCPP_ERROR(this->get_logger(), "FAILSAFE: %s", reason.c_str());
    std_msgs::msg::String r; r.data=reason; reason_pub_->publish(r);
    std_msgs::msg::Bool b; b.data=true; abort_pub_->publish(b);
  }

  double max_h_, arena_, vio_timeout_, lidar_timeout_, low_batt_, voltage_=16.8;
  geometry_msgs::msg::PoseStamped pose_;
  bool has_pose_=false, emergency_=false;
  rclcpp::Time last_vio_, last_lidar_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, vio_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr batt_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_, status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FailsafeNode>());
  rclcpp::shutdown();
  return 0;
}
