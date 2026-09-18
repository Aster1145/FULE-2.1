#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>

/*
 * mapping_node.cpp - C++ fast version for NIDAR AirMouse
 * Maintains 2D occupancy grid from RPLidar nodding + 3D voxel
 * Performance: O(n) raycasting, ~10x faster than Python
 * Ubuntu 22.04 + ROS2 Humble + C++17
 */

class MappingNode : public rclcpp::Node {
public:
  MappingNode() : Node("mapping_node") {
    this->declare_parameter("map_size", 20.0);
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("origin_x", -10.0);
    this->declare_parameter("origin_y", -10.0);
    this->declare_parameter("voxel_height", 3.0);

    map_size_ = this->get_parameter("map_size").as_double();
    res_ = this->get_parameter("resolution").as_double();
    origin_x_ = this->get_parameter("origin_x").as_double();
    origin_y_ = this->get_parameter("origin_y").as_double();
    voxel_h_ = this->get_parameter("voxel_height").as_double();

    width_ = static_cast<int>(map_size_ / res_);
    height_ = static_cast<int>(map_size_ / res_);
    z_dim_ = static_cast<int>(voxel_h_ / res_);

    grid_2d_.assign(height_ * width_, -1); // -1 unknown, 0 free, 100 occupied
    voxel_grid_.assign(z_dim_ * height_ * width_, -1);

    // Subs
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
    scan_2d_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_2d_projected", 10, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/offboard/pose", 10, std::bind(&MappingNode::poseCallback, this, std::placeholders::_1));
    vio_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vins_estimator/camera_pose", 10, std::bind(&MappingNode::poseCallback, this, std::placeholders::_1));

    // Pubs
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_map", 1);

    map_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MappingNode::publishMap, this));

    RCLCPP_INFO(this->get_logger(), "C++ Mapping Node: %dx%d grid, res %.2f m, voxel %d", width_, height_, res_, z_dim_);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    current_pose_ = *msg;
    has_pose_ = true;
  }

  std::pair<int,int> worldToGrid(double x, double y) {
    int gx = static_cast<int>((x - origin_x_) / res_);
    int gy = static_cast<int>((y - origin_y_) / res_);
    return {gx, gy};
  }

  void raycastAndUpdate(double x0, double y0, double x1, double y1, bool occupied) {
    auto [gx0, gy0] = worldToGrid(x0, y0);
    auto [gx1, gy1] = worldToGrid(x1, y1);

    int dx = std::abs(gx1 - gx0);
    int dy = std::abs(gy1 - gy0);
    int sx = (gx0 < gx1) ? 1 : -1;
    int sy = (gy0 < gy1) ? 1 : -1;
    int err = dx - dy;

    int x = gx0, y = gy0;
    int max_iter = dx + dy + 5;
    int iter = 0;
    while (iter < max_iter) {
      if (x >=0 && x < width_ && y >=0 && y < height_) {
        int idx = y * width_ + x;
        if (x == gx1 && y == gy1) {
          if (occupied) {
            grid_2d_[idx] = 100;
            int z_idx = static_cast<int>(0.5 / res_);
            if (z_idx >=0 && z_idx < z_dim_) {
              int v_idx = z_idx * height_ * width_ + y * width_ + x;
              voxel_grid_[v_idx] = 1;
              for (int z=0; z<z_idx; ++z) {
                voxel_grid_[z*height_*width_ + y*width_ + x] = 1;
              }
            }
          }
          break;
        } else {
          if (grid_2d_[idx] != 100) grid_2d_[idx] = 0;
          int z_idx = static_cast<int>(0.5 / res_);
          if (z_idx >=0 && z_idx < z_dim_) {
            int v_idx = z_idx * height_ * width_ + y * width_ + x;
            if (voxel_grid_[v_idx]==-1) voxel_grid_[v_idx]=0;
          }
        }
      } else break;

      int e2 = 2*err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 < dx) { err += dx; y += sy; }
      ++iter;
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double robot_x = 0, robot_y = 0, robot_yaw = 0;
    if (has_pose_) {
      robot_x = current_pose_.pose.position.x;
      robot_y = current_pose_.pose.position.y;
      auto q = current_pose_.pose.orientation;
      double siny_cosp = 2*(q.w*q.z + q.x*q.y);
      double cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z);
      robot_yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    double angle = msg->angle_min;
    for (size_t i=0; i<msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max) {
        angle += msg->angle_increment;
        continue;
      }
      double end_x = robot_x + r * std::cos(robot_yaw + angle);
      double end_y = robot_y + r * std::sin(robot_yaw + angle);
      raycastAndUpdate(robot_x, robot_y, end_x, end_y, true);
      angle += msg->angle_increment;
    }
  }

  void publishMap() {
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = this->now();
    out.header.frame_id = "map";
    out.info.resolution = res_;
    out.info.width = width_;
    out.info.height = height_;
    out.info.origin.position.x = origin_x_;
    out.info.origin.position.y = origin_y_;
    out.info.origin.position.z = 0;
    out.info.origin.orientation.w = 1.0;
    out.data = grid_2d_;
    map_pub_->publish(out);

    // Publish voxel as PointCloud2 (downsampled)
    std::vector<std::array<float,3>> points;
    points.reserve(5000);
    for (int z=0; z<z_dim_; ++z) {
      for (int y=0; y<height_; ++y) {
        for (int x=0; x<width_; ++x) {
          int idx = z*height_*width_ + y*width_ + x;
          if (voxel_grid_[idx]==1) {
            if (points.size() > 5000 && (rand()%2==0)) continue;
            float wx = x*res_ + origin_x_ + res_/2;
            float wy = y*res_ + origin_y_ + res_/2;
            float wz = z*res_;
            points.push_back({wx,wy,wz});
          }
        }
      }
    }
    if (!points.empty()) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.stamp = this->now();
      cloud.header.frame_id = "map";
      cloud.height = 1;
      cloud.width = points.size();
      cloud.fields.resize(3);
      cloud.fields[0].name="x"; cloud.fields[0].offset=0; cloud.fields[0].datatype=7; cloud.fields[0].count=1;
      cloud.fields[1].name="y"; cloud.fields[1].offset=4; cloud.fields[1].datatype=7; cloud.fields[1].count=1;
      cloud.fields[2].name="z"; cloud.fields[2].offset=8; cloud.fields[2].datatype=7; cloud.fields[2].count=1;
      cloud.point_step=12;
      cloud.row_step=cloud.point_step*points.size();
      cloud.is_dense=true;
      cloud.data.resize(cloud.row_step);
      for (size_t i=0;i<points.size();++i) {
        std::memcpy(&cloud.data[i*12], &points[i][0], 4);
        std::memcpy(&cloud.data[i*12+4], &points[i][1], 4);
        std::memcpy(&cloud.data[i*12+8], &points[i][2], 4);
      }
      voxel_pub_->publish(cloud);
    }
  }

  double map_size_, res_, origin_x_, origin_y_, voxel_h_;
  int width_, height_, z_dim_;
  std::vector<int8_t> grid_2d_;
  std::vector<int8_t> voxel_grid_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_pose_ = false;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_, scan_2d_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_, vio_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}
