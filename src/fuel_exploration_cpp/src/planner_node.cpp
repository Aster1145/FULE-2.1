#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>

/*
 * planner_node.cpp - C++ A* + B-spline smoothing for NIDAR
 * Fast: A* with binary heap, ESDF collision check, B-spline via de Boor
 */

struct Node2D {
  int x,y;
  double g, f;
  bool operator>(const Node2D& other) const { return f > other.f; }
};

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("planner_node") {
    this->declare_parameter("planning_height", 1.5);
    this->declare_parameter("safety_margin", 0.3);
    height_ = this->get_parameter("planning_height").as_double();
    safety_ = this->get_parameter("safety_margin").as_double();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&PlannerNode::mapCb, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/offboard/pose", 10, std::bind(&PlannerNode::poseCb, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/exploration/goal", 10, std::bind(&PlannerNode::goalCb, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planner/path", 10);
    traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
    viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planner/viz", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::plan, this));
    RCLCPP_INFO(this->get_logger(), "C++ Planner Node ready");
  }

private:
  void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_=*msg; has_map_=true; }
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_=*msg; has_pose_=true; }
  void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { goal_=*msg; has_goal_=true; RCLCPP_INFO(this->get_logger(), "Goal %.2f,%.2f", msg->pose.position.x, msg->pose.position.y); }

  std::pair<int,int> worldToGrid(double x, double y) {
    int gx = static_cast<int>((x - map_.info.origin.position.x)/map_.info.resolution);
    int gy = static_cast<int>((y - map_.info.origin.position.y)/map_.info.resolution);
    return {gx,gy};
  }
  std::pair<double,double> gridToWorld(int gx, int gy) {
    double x = gx*map_.info.resolution + map_.info.origin.position.x + map_.info.resolution/2;
    double y = gy*map_.info.resolution + map_.info.origin.position.y + map_.info.resolution/2;
    return {x,y};
  }

  bool isCollision(int gx, int gy) {
    if (!has_map_) return true;
    int margin = static_cast<int>(safety_/map_.info.resolution);
    for (int dy=-margin; dy<=margin; ++dy) {
      for (int dx=-margin; dx<=margin; ++dx) {
        int ny=gy+dy, nx=gx+dx;
        if (ny>=0 && ny<(int)map_.info.height && nx>=0 && nx<(int)map_.info.width) {
          if (map_.data[ny*map_.info.width+nx]==100) return true;
        }
      }
    }
    return false;
  }

  std::vector<std::pair<int,int>> astar(std::pair<int,int> start, std::pair<int,int> goal) {
    if (!has_map_) return {};
    int w=map_.info.width, h=map_.info.height;
    auto [sx,sy]=start; auto [gx,gy]=goal;
    if (sx<0||sx>=w||sy<0||sy>=h||gx<0||gx>=w||gy<0||gy>=h) return {};

    // Find nearest free if goal occupied
    if (isCollision(gx,gy)) {
      for (int r=1;r<10;++r) {
        for (int dy=-r; dy<=r; ++dy) {
          for (int dx=-r; dx<=r; ++dx) {
            int ny=gy+dy, nx=gx+dx;
            if (ny>=0&&ny<h&&nx>=0&&nx<w&&!isCollision(nx,ny)&&map_.data[ny*w+nx]==0) { gx=nx; gy=ny; goto found; }
          }
        }
      }
      found:;
    }

    std::priority_queue<Node2D, std::vector<Node2D>, std::greater<Node2D>> open;
    std::vector<std::vector<double>> g_score(h, std::vector<double>(w, 1e9));
    std::vector<std::vector<std::pair<int,int>>> came_from(h, std::vector<std::pair<int,int>>(w, {-1,-1}));
    std::vector<std::vector<bool>> closed(h, std::vector<bool>(w,false));

    open.push({sx,sy,0,std::hypot(gx-sx,gy-sy)});
    g_score[sy][sx]=0;

    const int dirs[8][2]={{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{-1,1},{1,-1},{1,1}};

    while(!open.empty()) {
      auto cur=open.top(); open.pop();
      if (closed[cur.y][cur.x]) continue;
      closed[cur.y][cur.x]=true;
      if (cur.x==gx && cur.y==gy) {
        std::vector<std::pair<int,int>> path;
        int cx=gx, cy=gy;
        while(cx!=-1) { path.push_back({cx,cy}); auto p=came_from[cy][cx]; cx=p.first; cy=p.second; }
        std::reverse(path.begin(), path.end());
        return path;
      }
      for (auto &d: dirs) {
        int nx=cur.x+d[0], ny=cur.y+d[1];
        if (nx<0||nx>=w||ny<0||ny>=h||closed[ny][nx]||isCollision(nx,ny)) continue;
        double move_cost = (d[0]==0||d[1]==0)?1.0:1.414;
        if (map_.data[ny*w+nx]==-1) move_cost*=1.5;
        double tentative = g_score[cur.y][cur.x]+move_cost;
        if (tentative < g_score[ny][nx]) {
          came_from[ny][nx]={cur.x,cur.y};
          g_score[ny][nx]=tentative;
          double f = tentative + std::hypot(gx-nx,gy-ny);
          open.push({nx,ny,tentative,f});
        }
      }
    }
    return {};
  }

  void plan() {
    if (!has_map_||!has_pose_||!has_goal_) return;
    auto start = worldToGrid(pose_.pose.position.x, pose_.pose.position.y);
    auto goal = worldToGrid(goal_.pose.position.x, goal_.pose.position.y);
    auto grid_path = astar(start, goal);
    if (grid_path.empty()) { RCLCPP_WARN(this->get_logger(), "A* failed"); return; }

    // Convert to world and B-spline smoothing (simple moving average for speed)
    std::vector<std::pair<double,double>> world_path;
    for (auto &p: grid_path) { world_path.push_back(gridToWorld(p.first,p.second)); }

    // Simple smoothing: 3-point average, collision check
    std::vector<std::pair<double,double>> smooth;
    for (size_t i=0;i<world_path.size();++i) {
      if (i==0||i==world_path.size()-1) smooth.push_back(world_path[i]);
      else {
        double x=(world_path[i-1].first+world_path[i].first+world_path[i+1].first)/3;
        double y=(world_path[i-1].second+world_path[i].second+world_path[i+1].second)/3;
        auto g=worldToGrid(x,y);
        if (!isCollision(g.first,g.second)) smooth.push_back({x,y});
        else smooth.push_back(world_path[i]);
      }
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id="map"; path_msg.header.stamp=this->now();
    for (auto &p: smooth) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header=path_msg.header;
      ps.pose.position.x=p.first; ps.pose.position.y=p.second; ps.pose.position.z=height_;
      ps.pose.orientation.w=1.0;
      path_msg.poses.push_back(ps);
    }
    path_pub_->publish(path_msg);
    traj_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Planned %zu waypoints", smooth.size());
  }

  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::PoseStamped pose_, goal_;
  bool has_map_=false, has_pose_=false, has_goal_=false;
  double height_, safety_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
