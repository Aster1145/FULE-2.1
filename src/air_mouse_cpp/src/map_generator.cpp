#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>
#include <fstream>

class MapGenerator : public rclcpp::Node {
public:
  MapGenerator() : Node("map_generator") {
    map_sub_=this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&MapGenerator::mapCb, this, std::placeholders::_1));
    surv_sub_=this->create_subscription<visualization_msgs::msg::MarkerArray>("/survivor_markers", 10, std::bind(&MapGenerator::survCb, this, std::placeholders::_1));
    timer_=this->create_wall_timer(std::chrono::seconds(5), std::bind(&MapGenerator::generate, this));
    RCLCPP_INFO(this->get_logger(), "C++ Map Generator");
  }

private:
  void mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_=*msg; has_map_=true; }
  void survCb(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    survivors_.clear();
    for (auto &m: msg->markers) if (m.ns=="survivors") survivors_.push_back(m);
  }

  void generate() {
    if (!has_map_) return;
    int w=map_.info.width, h=map_.info.height;
    cv::Mat img(h, w, CV_8UC3, cv::Scalar(128,128,128));
    for (int y=0; y<h; ++y) {
      for (int x=0; x<w; ++x) {
        int idx=y*w+x;
        int8_t v=map_.data[idx];
        if (v==0) img.at<cv::Vec3b>(y,x)=cv::Vec3b(255,255,255);
        else if (v==100) img.at<cv::Vec3b>(y,x)=cv::Vec3b(0,0,0);
      }
    }
    // Draw survivors
    for (auto &m: survivors_) {
      int gx = static_cast<int>((m.pose.position.x - map_.info.origin.position.x)/map_.info.resolution);
      int gy = static_cast<int>((m.pose.position.y - map_.info.origin.position.y)/map_.info.resolution);
      if (gx>=0&&gx<w&&gy>=0&&gy<h) {
        cv::circle(img, cv::Point(gx,gy), 8, cv::Scalar(0,165,255), -1);
        cv::putText(img, "S"+std::to_string(m.id+1), cv::Point(gx+10,gy), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255),2);
      }
    }
    // Home
    int home_gx = static_cast<int>((0 - map_.info.origin.position.x)/map_.info.resolution);
    int home_gy = static_cast<int>((0 - map_.info.origin.position.y)/map_.info.resolution);
    int size_px = static_cast<int>(0.61/map_.info.resolution);
    cv::rectangle(img, cv::Point(home_gx-size_px/2, home_gy-size_px/2), cv::Point(home_gx+size_px/2, home_gy+size_px/2), cv::Scalar(0,255,0),2);

    cv::Mat flipped; cv::flip(img, flipped, 0);
    cv::imwrite("/tmp/nidar_2d_map_with_survivors.png", flipped);
    cv::imwrite("/tmp/nidar_2d_map.png", flipped);

    // PGM
    cv::Mat gray; cv::cvtColor(flipped, gray, cv::COLOR_BGR2GRAY);
    cv::Mat pgm(h,w,CV_8UC1);
    for (int y=0;y<h;++y) for (int x=0;x<w;++x) {
      int idx=(h-1-y)*w+x; // flip back for pgm
      int8_t v=map_.data[idx];
      if (v==0) pgm.at<uchar>(y,x)=254;
      else if (v==100) pgm.at<uchar>(y,x)=0;
      else pgm.at<uchar>(y,x)=205;
    }
    cv::imwrite("/tmp/nidar_2d_map.pgm", pgm);

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "image" << YAML::Value << "/tmp/nidar_2d_map.pgm";
    out << YAML::Key << "resolution" << YAML::Value << map_.info.resolution;
    out << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq << map_.info.origin.position.x << map_.info.origin.position.y << 0.0 << YAML::EndSeq;
    out << YAML::Key << "negate" << YAML::Value << 0;
    out << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
    out << YAML::Key << "free_thresh" << YAML::Value << 0.196;
    out << YAML::EndMap;
    std::ofstream yaml("/tmp/nidar_2d_map.yaml");
    yaml<<out.c_str();

    RCLCPP_INFO(this->get_logger(), "Map saved with %zu survivors", survivors_.size());
  }

  nav_msgs::msg::OccupancyGrid map_;
  bool has_map_=false;
  std::vector<visualization_msgs::msg::Marker> survivors_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr surv_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapGenerator>());
  rclcpp::shutdown();
  return 0;
}
