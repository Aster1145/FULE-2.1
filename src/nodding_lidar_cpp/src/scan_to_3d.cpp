#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <deque>
#include <cmath>
#include <cstring>

struct ScanEntry {
  sensor_msgs::msg::LaserScan scan;
  double pitch;
  geometry_msgs::msg::PoseStamped pose;
  bool has_pose;
};

class ScanTo3D : public rclcpp::Node {
public:
  ScanTo3D() : Node("scan_to_3d") {
    this->declare_parameter("accumulate_scans", 20);
    this->declare_parameter("min_range", 0.15);
    this->declare_parameter("max_range", 12.0);

    accumulate_n_ = this->get_parameter("accumulate_scans").as_int();
    min_range_ = this->get_parameter("min_range").as_double();
    max_range_ = this->get_parameter("max_range").as_double();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&ScanTo3D::scanCb, this, std::placeholders::_1));
    pitch_sub_ = this->create_subscription<std_msgs::msg::Float64>("/servo_angle_cmd", 10, std::bind(&ScanTo3D::pitchCb, this, std::placeholders::_1));
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&ScanTo3D::jointCb, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/offboard/pose", 10, std::bind(&ScanTo3D::poseCb, this, std::placeholders::_1));

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_3d", 10);
    cloud_world_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_3d_world", 10);

    RCLCPP_INFO(this->get_logger(), "C++ ScanTo3D accumulate %d", accumulate_n_);
  }

private:
  void pitchCb(const std_msgs::msg::Float64::SharedPtr msg) { current_pitch_=msg->data; }
  void jointCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i=0;i<msg->name.size();++i) if (msg->name[i]=="rplidar_pitch_joint") current_pitch_=msg->position[i];
  }
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pose_=*msg; has_pose_=true; }

  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    ScanEntry e; e.scan=*msg; e.pitch=current_pitch_; e.pose=current_pose_; e.has_pose=has_pose_;
    buffer_.push_back(e);
    if ((int)buffer_.size() > accumulate_n_) buffer_.pop_front();
    if ((int)buffer_.size() >= accumulate_n_) generate();
  }

  void generate() {
    std::vector<std::array<float,3>> local_pts, world_pts;
    local_pts.reserve(10000); world_pts.reserve(10000);

    for (auto &entry: buffer_) {
      double pitch=entry.pitch;
      double cos_p=std::cos(pitch), sin_p=std::sin(pitch);
      double angle=entry.scan.angle_min;
      double yaw=0;
      if (entry.has_pose) {
        auto q=entry.pose.pose.orientation;
        double siny=2*(q.w*q.z+q.x*q.y);
        double cosy=1-2*(q.y*q.y+q.z*q.z);
        yaw=std::atan2(siny,cosy);
      }
      double cos_y=std::cos(yaw), sin_y=std::sin(yaw);

      for (size_t i=0;i<entry.scan.ranges.size();++i) {
        float r=entry.scan.ranges[i];
        if (std::isinf(r)||std::isnan(r)||r<min_range_||r>max_range_) { angle+=entry.scan.angle_increment; continue; }
        double x_lidar = r*std::cos(angle);
        double y_lidar = r*std::sin(angle);
        double x_mount = x_lidar*cos_p;
        double y_mount = y_lidar;
        double z_mount = -x_lidar*sin_p;

        double x_base=x_mount, y_base=y_mount, z_base=z_mount+0.12;
        local_pts.push_back({(float)x_base,(float)y_base,(float)z_base});

        if (entry.has_pose) {
          double x_world = entry.pose.pose.position.x + x_base*cos_y - y_base*sin_y;
          double y_world = entry.pose.pose.position.y + x_base*sin_y + y_base*cos_y;
          double z_world = entry.pose.pose.position.z + z_base;
          world_pts.push_back({(float)x_world,(float)y_world,(float)z_world});
        }
        angle+=entry.scan.angle_increment;
      }
    }

    publishCloud(local_pts, cloud_pub_, "base_link");
    publishCloud(world_pts, cloud_world_pub_, "map");

    if (!buffer_.empty()) {
      double min_p=buffer_[0].pitch, max_p=buffer_[0].pitch;
      for (auto &e: buffer_) { min_p=std::min(min_p,e.pitch); max_p=std::max(max_p,e.pitch); }
      RCLCPP_INFO(this->get_logger(), "3D cloud %zu local %zu world pitch %.2f to %.2f rad", local_pts.size(), world_pts.size(), min_p, max_p);
    }
  }

  void publishCloud(const std::vector<std::array<float,3>>& pts, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, const std::string& frame) {
    if (pts.empty()) return;
    // Downsample to 10k
    std::vector<std::array<float,3>> filtered;
    if (pts.size()>10000) {
      int step=pts.size()/10000;
      for (size_t i=0;i<pts.size();i+=step) filtered.push_back(pts[i]);
    } else filtered=pts;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp=this->now(); cloud.header.frame_id=frame;
    cloud.height=1; cloud.width=filtered.size();
    cloud.fields.resize(3);
    cloud.fields[0].name="x"; cloud.fields[0].offset=0; cloud.fields[0].datatype=7; cloud.fields[0].count=1;
    cloud.fields[1].name="y"; cloud.fields[1].offset=4; cloud.fields[1].datatype=7; cloud.fields[1].count=1;
    cloud.fields[2].name="z"; cloud.fields[2].offset=8; cloud.fields[2].datatype=7; cloud.fields[2].count=1;
    cloud.point_step=12; cloud.row_step=cloud.point_step*filtered.size(); cloud.is_dense=true;
    cloud.data.resize(cloud.row_step);
    for (size_t i=0;i<filtered.size();++i) {
      std::memcpy(&cloud.data[i*12], &filtered[i][0], 4);
      std::memcpy(&cloud.data[i*12+4], &filtered[i][1], 4);
      std::memcpy(&cloud.data[i*12+8], &filtered[i][2], 4);
    }
    pub->publish(cloud);
  }

  int accumulate_n_;
  double min_range_, max_range_, current_pitch_=0;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_pose_=false;
  std::deque<ScanEntry> buffer_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_, cloud_world_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanTo3D>());
  rclcpp::shutdown();
  return 0;
}
