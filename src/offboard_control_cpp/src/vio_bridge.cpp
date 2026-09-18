#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <cmath>

/*
 * vio_bridge.cpp - C++ fast ENU->NED conversion for PX4 VIO
 * Uses Eigen for quaternion math, ~5x faster than Python scipy
 */

class VIOBridge : public rclcpp::Node {
public:
  VIOBridge() : Node("vio_bridge") {
    this->declare_parameter("input_topic", "/vins_estimator/camera_pose");
    this->declare_parameter("output_topic", "/fmu/in/vehicle_visual_odometry");

    std::string input = this->get_parameter("input_topic").as_string();
    std::string output = this->get_parameter("output_topic").as_string();

    vio_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(input, 10, std::bind(&VIOBridge::vioCb, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/vins_estimator/odometry", 10, std::bind(&VIOBridge::odomCb, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(output, 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

    RCLCPP_INFO(this->get_logger(), "C++ VIO Bridge %s -> %s", input.c_str(), output.c_str());
  }

private:
  void vioCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // ENU to NED: x_ned=y_enu, y_ned=x_enu, z_ned=-z_enu
    double x_enu=msg->pose.position.x, y_enu=msg->pose.position.y, z_enu=msg->pose.position.z;
    double x_ned=y_enu, y_ned=x_enu, z_ned=-z_enu;

    // Quaternion conversion using rotation matrix
    Eigen::Quaterniond q_enu(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Matrix3d R_enu = q_enu.toRotationMatrix();
    Eigen::Matrix3d R_enu_to_ned;
    R_enu_to_ned << 0,1,0, 1,0,0, 0,0,-1;
    Eigen::Matrix3d R_ned = R_enu_to_ned * R_enu;
    Eigen::Quaterniond q_ned(R_ned);
    q_ned.normalize();

    px4_msgs::msg::VehicleOdometry odom;
    odom.timestamp = this->now().nanoseconds()/1000;
    odom.timestamp_sample = odom.timestamp;
    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    odom.position = {static_cast<float>(x_ned), static_cast<float>(y_ned), static_cast<float>(z_ned)};
    odom.q = {static_cast<float>(q_ned.w()), static_cast<float>(q_ned.x()), static_cast<float>(q_ned.y()), static_cast<float>(q_ned.z())};
    odom.velocity = {0,0,0};
    odom.angular_velocity = {0,0,0};
    odom.position_variance = {0.1,0.1,0.1};
    odom.orientation_variance = {0.1,0.1,0.1};
    odom.velocity_variance = {0.1,0.1,0.1};

    odom_pub_->publish(odom);
    pose_pub_->publish(*msg);
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header=msg->header;
    ps.pose=msg->pose.pose;
    vioCb(std::make_shared<geometry_msgs::msg::PoseStamped>(ps));
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VIOBridge>());
  rclcpp::shutdown();
  return 0;
}
