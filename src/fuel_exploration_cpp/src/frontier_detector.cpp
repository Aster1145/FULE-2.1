#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

/*
 * frontier_detector.cpp - C++ FIS for NIDAR
 * Fast frontier detection using BFS clustering, info gain, cost sorting
 * 10-20x faster than Python due to direct grid access
 */

struct Frontier {
  double cx, cy;
  std::vector<std::pair<int,int>> cells;
  int gain;
  double cost;
};

class FrontierDetector : public rclcpp::Node {
public:
  FrontierDetector() : Node("frontier_detector") {
    this->declare_parameter("min_frontier_size", 8);
    this->declare_parameter("sensor_range", 4.0);
    this->declare_parameter("info_gain_threshold", 10);

    min_size_ = this->get_parameter("min_frontier_size").as_int();
    sensor_range_ = this->get_parameter("sensor_range").as_double();
    gain_thresh_ = this->get_parameter("info_gain_threshold").as_int();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&FrontierDetector::mapCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/offboard/pose", 10, std::bind(&FrontierDetector::poseCallback, this, std::placeholders::_1));

    frontier_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/frontiers", 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/exploration/goal", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FrontierDetector::detect, this));

    RCLCPP_INFO(this->get_logger(), "C++ Frontier Detector (FIS) ready");
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    has_pose_ = true;
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    has_map_ = true;
    width_ = msg->info.width;
    height_ = msg->info.height;
  }

  bool isFrontierCell(int y, int x) {
    if (y<=0 || y>=height_-1 || x<=0 || x>=width_-1) return false;
    int idx = y*width_ + x;
    if (map_.data[idx] != 0) return false; // must be free
    for (int dy=-1; dy<=1; ++dy) {
      for (int dx=-1; dx<=1; ++dx) {
        if (dx==0 && dy==0) continue;
        int ny=y+dy, nx=x+dx;
        int nidx = ny*width_+nx;
        if (map_.data[nidx]==-1) return true;
      }
    }
    return false;
  }

  int computeGain(double wx, double wy) {
    if (!has_map_) return 0;
    int gx = static_cast<int>((wx - map_.info.origin.position.x)/map_.info.resolution);
    int gy = static_cast<int>((wy - map_.info.origin.position.y)/map_.info.resolution);
    int r = static_cast<int>(sensor_range_/map_.info.resolution);
    int gain=0;
    for (int dy=-r; dy<=r; ++dy) {
      for (int dx=-r; dx<=r; ++dx) {
        if (dx*dx+dy*dy > r*r) continue;
        int ny=gy+dy, nx=gx+dx;
        if (ny>=0 && ny<height_ && nx>=0 && nx<width_) {
          if (map_.data[ny*width_+nx]==-1) ++gain;
        }
      }
    }
    return gain;
  }

  void detect() {
    if (!has_map_) return;

    std::vector<std::vector<bool>> visited(height_, std::vector<bool>(width_, false));
    std::vector<Frontier> frontiers;

    for (int y=1; y<height_-1; ++y) {
      for (int x=1; x<width_-1; ++x) {
        if (visited[y][x]) continue;
        if (isFrontierCell(y,x)) {
          // BFS cluster
          std::vector<std::pair<int,int>> cluster;
          std::queue<std::pair<int,int>> q;
          q.push({y,x});
          visited[y][x]=true;
          while(!q.empty()) {
            auto [cy,cx]=q.front(); q.pop();
            cluster.push_back({cy,cx});
            for (int dy=-1; dy<=1; ++dy) {
              for (int dx=-1; dx<=1; ++dx) {
                int ny=cy+dy, nx=cx+dx;
                if (ny>=0 && ny<height_ && nx>=0 && nx<width_ && !visited[ny][nx]) {
                  visited[ny][nx]=true;
                  if (isFrontierCell(ny,nx)) q.push({ny,nx});
                }
              }
            }
          }
          if ((int)cluster.size() >= min_size_) {
            double sum_x=0, sum_y=0;
            for (auto &p: cluster) {
              double wx = p.second*map_.info.resolution + map_.info.origin.position.x;
              double wy = p.first*map_.info.resolution + map_.info.origin.position.y;
              sum_x+=wx; sum_y+=wy;
            }
            Frontier f;
            f.cx = sum_x/cluster.size();
            f.cy = sum_y/cluster.size();
            f.cells = cluster;
            f.gain = computeGain(f.cx, f.cy);
            f.cost = -f.gain;
            if (has_pose_) {
              double dx = f.cx - current_pose_.pose.position.x;
              double dy = f.cy - current_pose_.pose.position.y;
              double dist = std::sqrt(dx*dx+dy*dy);
              f.cost += 0.5*dist;
            }
            frontiers.push_back(f);
          }
        }
      }
    }

    std::sort(frontiers.begin(), frontiers.end(), [](const Frontier& a, const Frontier& b){return a.cost < b.cost;});

    // Publish markers
    visualization_msgs::msg::MarkerArray ma;
    int id=0;
    for (auto &f: frontiers) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id="map"; m.header.stamp=this->now();
      m.ns="frontiers"; m.id=id++; m.type=3; m.action=0;
      m.pose.position.x=f.cx; m.pose.position.y=f.cy; m.pose.position.z=0.5;
      m.pose.orientation.w=1.0;
      m.scale.x=0.3; m.scale.y=0.3; m.scale.z=0.3;
      m.color.r=1.0 - std::min(f.gain/50.0,1.0); m.color.g=std::min(f.gain/50.0,1.0); m.color.b=0.0; m.color.a=0.8;
      ma.markers.push_back(m);
    }
    frontier_pub_->publish(ma);

    if (!frontiers.empty() && frontiers[0].gain >= gain_thresh_) {
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id="map"; goal.header.stamp=this->now();
      goal.pose.position.x=frontiers[0].cx;
      goal.pose.position.y=frontiers[0].cy;
      goal.pose.position.z=1.5;
      goal.pose.orientation.w=1.0;
      goal_pub_->publish(goal);
      RCLCPP_INFO(this->get_logger(), "Frontier best gain %d at %.2f,%.2f (%zu total)", frontiers[0].gain, frontiers[0].cx, frontiers[0].cy, frontiers.size());
    }
  }

  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_map_=false, has_pose_=false;
  int width_, height_;
  int min_size_, gain_thresh_;
  double sensor_range_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierDetector>());
  rclcpp::shutdown();
  return 0;
}
