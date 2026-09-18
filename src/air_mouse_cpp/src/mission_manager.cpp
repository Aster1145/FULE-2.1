#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <chrono>

struct Survivor {
  int id;
  double x,y,z;
  double time;
};

class MissionManager : public rclcpp::Node {
public:
  MissionManager() : Node("mission_manager") {
    this->declare_parameter("arena_size", 15.0);
    this->declare_parameter("max_mission_time", 1800.0);
    this->declare_parameter("max_survivors", 6);
    this->declare_parameter("home_x", 0.0);
    this->declare_parameter("home_y", 0.0);

    arena_=this->get_parameter("arena_size").as_double();
    max_time_=this->get_parameter("max_mission_time").as_double();
    max_surv_=this->get_parameter("max_survivors").as_int();
    home_x_=this->get_parameter("home_x").as_double();
    home_y_=this->get_parameter("home_y").as_double();

    map_sub_=this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&MissionManager::mapCb, this, std::placeholders::_1));
    pose_sub_=this->create_subscription<geometry_msgs::msg::PoseStamped>("/offboard/pose", 10, std::bind(&MissionManager::poseCb, this, std::placeholders::_1));
    surv_sub_=this->create_subscription<geometry_msgs::msg::PoseStamped>("/person_detections", 10, std::bind(&MissionManager::survCb, this, std::placeholders::_1));
    e_stop_sub_=this->create_subscription<std_msgs::msg::Bool>("/emergency_stop", 10, std::bind(&MissionManager::eStopCb, this, std::placeholders::_1));

    state_pub_=this->create_publisher<std_msgs::msg::String>("/mission/state", 10);
    time_pub_=this->create_publisher<std_msgs::msg::Float32>("/mission/time_elapsed", 10);
    surv_list_pub_=this->create_publisher<std_msgs::msg::String>("/mission/survivors", 10);
    goal_pub_=this->create_publisher<geometry_msgs::msg::PoseStamped>("/exploration/goal", 10);
    abort_pub_=this->create_publisher<std_msgs::msg::Bool>("/mission/abort", 10);

    state_="IDLE";
    start_time_=this->now();
    timer_=this->create_wall_timer(std::chrono::seconds(1), std::bind(&MissionManager::fsm, this));
    RCLCPP_INFO(this->get_logger(), "C++ AirMouse Mission Manager arena %.1f max %.0fs", arena_, max_time_);
  }

private:
  void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    int unknown=0;
    for (auto v: msg->data) if (v==-1) ++unknown;
    explored_=1.0 - (double)unknown/msg->data.size();
  }
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_=*msg; has_pose_=true; }
  void survCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Check duplicate 1m
    for (auto &s: survivors_) {
      if (std::hypot(s.x-msg->pose.position.x, s.y-msg->pose.position.y)<1.0) return;
    }
    if ((int)survivors_.size()>=max_surv_) return;
    Survivor sv;
    sv.id=survivors_.size()+1;
    sv.x=msg->pose.position.x; sv.y=msg->pose.position.y; sv.z=msg->pose.position.z;
    sv.time=(this->now()-start_time_).seconds();
    survivors_.push_back(sv);
    RCLCPP_INFO(this->get_logger(), "Survivor %d at %.2f,%.2f total %zu/%d", sv.id, sv.x, sv.y, survivors_.size(), max_surv_);
    // Save json
    std::ofstream f("/tmp/nidar_survivors.json");
    f<<"[\n";
    for (size_t i=0;i<survivors_.size();++i) {
      f<<"  {\"id\":"<<survivors_[i].id<<",\"x\":"<<survivors_[i].x<<",\"y\":"<<survivors_[i].y<<"}";
      if (i+1<survivors_.size()) f<<",";
      f<<"\n";
    }
    f<<"]\n";
  }
  void eStopCb(const std_msgs::msg::Bool::SharedPtr msg) { if (msg->data) { emergency_=true; state_="ABORT"; } }

  void fsm() {
    std_msgs::msg::String s; s.data=state_; state_pub_->publish(s);
    double elapsed=(this->now()-start_time_).seconds();
    std_msgs::msg::Float32 t; t.data=elapsed; time_pub_->publish(t);
    if (elapsed>max_time_ && state_!="RETURN_HOME" && state_!="LAND" && state_!="COMPLETE") { state_="RETURN_HOME"; }

    if (state_=="IDLE") { state_="SETUP"; setup_time_=this->now(); }
    else if (state_=="SETUP") { if ((this->now()-setup_time_).seconds()>5.0) state_="ARM"; }
    else if (state_=="ARM") { state_="TAKEOFF"; takeoff_time_=this->now(); start_time_=this->now(); }
    else if (state_=="TAKEOFF") { if ((this->now()-takeoff_time_).seconds()>5.0) state_="EXPLORE_MAZE"; }
    else if (state_=="EXPLORE_MAZE") {
      if ((int)survivors_.size()>=max_surv_) state_="GENERATE_MAP";
      else if (explored_>0.90) state_="GENERATE_MAP";
    }
    else if (state_=="GENERATE_MAP") { state_="RETURN_HOME"; }
    else if (state_=="RETURN_HOME") {
      geometry_msgs::msg::PoseStamped home;
      home.header.frame_id="map"; home.header.stamp=this->now();
      home.pose.position.x=home_x_; home.pose.position.y=home_y_; home.pose.position.z=1.5; home.pose.orientation.w=1.0;
      goal_pub_->publish(home);
      if (has_pose_) {
        double dx=pose_.pose.position.x-home_x_, dy=pose_.pose.position.y-home_y_;
        if (std::hypot(dx,dy)<0.5) { state_="LAND"; saveReport(); }
      }
    }
    else if (state_=="LAND") { state_="COMPLETE"; }
    else if (state_=="ABORT") {
      std_msgs::msg::Bool b; b.data=true; abort_pub_->publish(b);
      state_="LAND";
    }
  }

  void saveReport() {
    std::ofstream f("/tmp/nidar_final_report.json");
    f<<"{\n";
    f<<"  \"mission\": \"NIDAR AirMouse C++\",\n";
    f<<"  \"arena_size\": "<<arena_<<",\n";
    f<<"  \"survivors_found\": "<<survivors_.size()<<",\n";
    f<<"  \"time\": "<<(this->now()-start_time_).seconds()<<"\n";
    f<<"}\n";
    RCLCPP_INFO(this->get_logger(), "Final report saved");
  }

  std::string state_;
  double arena_, max_time_, home_x_, home_y_, explored_=0;
  int max_surv_;
  geometry_msgs::msg::PoseStamped pose_;
  bool has_pose_=false, emergency_=false;
  std::vector<Survivor> survivors_;
  rclcpp::Time start_time_, setup_time_, takeoff_time_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, surv_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, surv_list_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
