#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <fstream>
#include <chrono>

class ExplorationManager : public rclcpp::Node {
public:
  ExplorationManager() : Node("exploration_manager") {
    this->declare_parameter("exploration_timeout", 600.0);
    this->declare_parameter("home_x", 0.0);
    this->declare_parameter("home_y", 0.0);

    timeout_ = this->get_parameter("exploration_timeout").as_double();
    home_x_ = this->get_parameter("home_x").as_double();
    home_y_ = this->get_parameter("home_y").as_double();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/offboard/pose", 10, std::bind(&ExplorationManager::poseCb, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/exploration/goal", 10, std::bind(&ExplorationManager::goalCb, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&ExplorationManager::mapCb, this, std::placeholders::_1));
    person_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/person_detections", 10, std::bind(&ExplorationManager::personCb, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<std_msgs::msg::String>("/exploration/state", 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/exploration/goal", 10);

    state_ = "INIT";
    start_time_ = this->now();
    last_goal_time_ = this->now();

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ExplorationManager::fsm, this));
    RCLCPP_INFO(this->get_logger(), "C++ Exploration Manager");
  }

private:
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_=*msg; has_pose_=true; }
  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { goal_=*msg; has_goal_=true; last_goal_time_=this->now(); }
  void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int unknown=0;
    for (auto v: msg->data) if (v==-1) ++unknown;
    explored_ratio_ = 1.0 - (double)unknown/msg->data.size();
  }
  void personCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    person_detections_.push_back(*msg);
    RCLCPP_INFO(this->get_logger(), "Person at %.2f,%.2f total %zu", msg->pose.position.x, msg->pose.position.y, person_detections_.size());
  }

  void fsm() {
    std_msgs::msg::String s; s.data=state_; state_pub_->publish(s);
    double elapsed = (this->now()-start_time_).seconds();

    if (state_=="INIT") { RCLCPP_INFO(this->get_logger(), "INIT->TAKEOFF"); state_="TAKEOFF"; takeoff_time_=this->now(); }
    else if (state_=="TAKEOFF") {
      if ((this->now()-takeoff_time_).seconds()>5.0) { state_="EXPLORING"; RCLCPP_INFO(this->get_logger(), "Takeoff done -> EXPLORING"); }
    }
    else if (state_=="EXPLORING") {
      if (elapsed>timeout_) { state_="RETURN_HOME"; return; }
      if (explored_ratio_>0.95) { state_="RETURN_HOME"; return; }
      if (has_pose_&&has_goal_) {
        double dx=pose_.pose.position.x-goal_.pose.position.x;
        double dy=pose_.pose.position.y-goal_.pose.position.y;
        if (std::hypot(dx,dy)<0.5) { state_="GOAL_REACHED"; }
      }
    }
    else if (state_=="GOAL_REACHED") { state_="EXPLORING"; last_goal_time_=this->now(); }
    else if (state_=="RETURN_HOME") {
      geometry_msgs::msg::PoseStamped home;
      home.header.frame_id="map"; home.header.stamp=this->now();
      home.pose.position.x=home_x_; home.pose.position.y=home_y_; home.pose.position.z=1.5; home.pose.orientation.w=1.0;
      goal_pub_->publish(home);
      if (has_pose_) {
        double dx=pose_.pose.position.x-home_x_, dy=pose_.pose.position.y-home_y_;
        if (std::hypot(dx,dy)<0.5) { state_="LAND"; saveResults(); }
      }
    }
    else if (state_=="LAND") { state_="FINISHED"; }
  }

  void saveResults() {
    std::ofstream f("/tmp/person_detections.json");
    f<<"[\n";
    for (size_t i=0;i<person_detections_.size();++i) {
      auto &p=person_detections_[i];
      f<<"  {\"x\":"<<p.pose.position.x<<",\"y\":"<<p.pose.position.y<<"}";
      if (i+1<person_detections_.size()) f<<",";
      f<<"\n";
    }
    f<<"]\n";
    RCLCPP_INFO(this->get_logger(), "Saved %zu detections", person_detections_.size());
  }

  std::string state_;
  double timeout_, home_x_, home_y_, explored_ratio_=0;
  geometry_msgs::msg::PoseStamped pose_, goal_;
  bool has_pose_=false, has_goal_=false;
  std::vector<geometry_msgs::msg::PoseStamped> person_detections_;
  rclcpp::Time start_time_, takeoff_time_, last_goal_time_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, goal_sub_, person_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationManager>());
  rclcpp::shutdown();
  return 0;
}
