#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>

class OscillationController : public rclcpp::Node {
public:
  OscillationController() : Node("oscillation_controller") {
    this->declare_parameter("min_angle_deg", -45.0);
    this->declare_parameter("max_angle_deg", 45.0);
    this->declare_parameter("frequency_hz", 0.5);
    this->declare_parameter("mode", "sinusoidal");

    double min_deg = this->get_parameter("min_angle_deg").as_double();
    double max_deg = this->get_parameter("max_angle_deg").as_double();
    freq_ = this->get_parameter("frequency_hz").as_double();
    mode_ = this->get_parameter("mode").as_string();

    min_angle_ = min_deg * M_PI/180.0;
    max_angle_ = max_deg * M_PI/180.0;

    angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("/servo_angle_cmd", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    enable_pub_ = this->create_publisher<std_msgs::msg::Bool>("/nodding_scan_enable", 10);

    start_time_ = this->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&OscillationController::update, this));

    RCLCPP_INFO(this->get_logger(), "C++ Nodding Controller %.1f to %.1f deg @ %.2f Hz", min_deg, max_deg, freq_);
  }

private:
  void update() {
    double elapsed = (this->now() - start_time_).seconds();
    double angle;
    if (mode_=="sinusoidal") {
      double amp = (max_angle_ - min_angle_)/2;
      double center = (max_angle_ + min_angle_)/2;
      angle = center + amp * std::sin(2*M_PI*freq_*elapsed);
    } else {
      double period = 1.0/freq_;
      double phase = std::fmod(elapsed, period)/period;
      if (phase<0.5) angle = min_angle_ + (max_angle_-min_angle_)*(phase*2);
      else angle = max_angle_ - (max_angle_-min_angle_)*((phase-0.5)*2);
    }

    std_msgs::msg::Float64 msg; msg.data=angle; angle_pub_->publish(msg);

    sensor_msgs::msg::JointState js;
    js.header.stamp=this->now();
    js.name={"rplidar_pitch_joint"};
    js.position={angle};
    js.velocity={0};
    joint_pub_->publish(js);

    std_msgs::msg::Bool en; en.data=true; enable_pub_->publish(en);
  }

  double min_angle_, max_angle_, freq_;
  std::string mode_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OscillationController>());
  rclcpp::shutdown();
  return 0;
}
