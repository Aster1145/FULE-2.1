#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <cmath>

/*
 * survivor_detector.cpp - C++ fast YOLO survivor detection for NIDAR
 * Uses OpenCV DNN with YOLOv8 ONNX model for speed on CPU/GPU
 * For competition: convert yolov8n.pt -> yolov8n.onnx via ultralytics, then run here
 * Much faster than Python ultralytics: no Python overhead, OpenCV optimized
 */

class SurvivorDetector : public rclcpp::Node {
public:
  SurvivorDetector() : Node("survivor_detector") {
    this->declare_parameter("model", "yolov8n.onnx");
    this->declare_parameter("conf_thres", 0.4);
    this->declare_parameter("image_topic", "/camera/rgb/image_raw");
    this->declare_parameter("max_survivors", 6);

    std::string model_path = this->get_parameter("model").as_string();
    conf_thresh_ = this->get_parameter("conf_thres").as_double();
    max_surv_ = this->get_parameter("max_survivors").as_int();

    try {
      net_ = cv::dnn::readNet(model_path);
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      RCLCPP_INFO(this->get_logger(), "YOLO ONNX loaded %s", model_path.c_str());
      has_model_=true;
    } catch (cv::Exception &e) {
      RCLCPP_WARN(this->get_logger(), "Failed to load ONNX %s: %s, using dummy", model_path.c_str(), e.what());
      has_model_=false;
    }

    image_sub_=this->create_subscription<sensor_msgs::msg::Image>(this->get_parameter("image_topic").as_string(), 10, std::bind(&SurvivorDetector::imageCb, this, std::placeholders::_1));
    pose_sub_=this->create_subscription<geometry_msgs::msg::PoseStamped>("/offboard/pose", 10, std::bind(&SurvivorDetector::poseCb, this, std::placeholders::_1));
    info_sub_=this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/rgb/camera_info", 10, std::bind(&SurvivorDetector::infoCb, this, std::placeholders::_1));

    marker_pub_=this->create_publisher<visualization_msgs::msg::MarkerArray>("/survivor_markers", 10);
    det_pub_=this->create_publisher<geometry_msgs::msg::PoseStamped>("/person_detections", 10);
    annotated_pub_=this->create_publisher<sensor_msgs::msg::Image>("/survivor/annotated", 10);

    RCLCPP_INFO(this->get_logger(), "C++ Survivor Detector max %d conf %.2f", max_surv_, conf_thresh_);
  }

private:
  void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_=*msg; has_pose_=true; }
  void infoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) { info_=*msg; has_info_=true; }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!has_model_) return;
    cv_bridge::CvImagePtr cv_ptr;
    try { cv_ptr=cv_bridge::toCvCopy(msg, "bgr8"); } catch (...) { return; }
    cv::Mat img=cv_ptr->image;
    int img_w=img.cols, img_h=img.rows;

    // Preprocess for YOLOv8: 640x640 letterbox
    cv::Mat blob;
    cv::dnn::blobFromImage(img, blob, 1/255.0, cv::Size(640,640), cv::Scalar(), true, false);
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    // YOLOv8 output: 1x84x8400 (4 box + 80 classes)
    // For person class 0
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;

    if (!outputs.empty()) {
      cv::Mat out = outputs[0]; // 1x84x8400
      // Transpose to 8400x84
      cv::Mat out_t;
      // out shape may be 1x84x8400, reshape
      int rows=84, cols=8400;
      if (out.total()==rows*cols) {
        cv::Mat flat = out.reshape(1, rows); // 84x8400
        cv::transpose(flat, out_t); // 8400x84
        for (int i=0;i<out_t.rows;++i) {
          float* data = (float*)out_t.row(i).data;
          float obj_conf = data[4]; // actually class 0 confidence for YOLOv8 is data[4] is person
          // YOLOv8: first 4 are box, rest classes
          float max_class=0; int class_id=0;
          for (int c=0;c<80;++c) {
            if (data[4+c]>max_class) { max_class=data[4+c]; class_id=c; }
          }
          if (class_id==0 && max_class>conf_thresh_) {
            float cx=data[0], cy=data[1], w=data[2], h=data[3];
            // Convert from 640 to original
            float x = (cx - w/2) * img_w/640;
            float y = (cy - h/2) * img_h/640;
            float bw = w * img_w/640;
            float bh = h * img_h/640;
            boxes.push_back(cv::Rect(x,y,bw,bh));
            confs.push_back(max_class);
          }
        }
      }
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confs, conf_thresh_, 0.5, indices);

    visualization_msgs::msg::MarkerArray ma;
    cv::Mat annotated=img.clone();

    for (int idx: indices) {
      auto box=boxes[idx];
      float conf=confs[idx];
      cv::rectangle(annotated, box, cv::Scalar(0,255,255),2);
      cv::putText(annotated, "Survivor "+std::to_string(conf).substr(0,4), cv::Point(box.x, box.y-10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,255),2);

      if (has_pose_) {
        double world_x, world_y, dist;
        estimateWorld(box, img_w, img_h, world_x, world_y, dist);

        bool is_new=true;
        for (auto &s: survivors_) if (std::hypot(s.first-world_x, s.second-world_y)<1.0) is_new=false;
        if (is_new && (int)survivors_.size()<max_surv_) {
          survivors_.push_back({world_x, world_y});

          geometry_msgs::msg::PoseStamped det;
          det.header.frame_id="map"; det.header.stamp=this->now();
          det.pose.position.x=world_x; det.pose.position.y=world_y; det.pose.position.z=0.9; det.pose.orientation.w=1.0;
          det_pub_->publish(det);

          visualization_msgs::msg::Marker m;
          m.header.frame_id="map"; m.header.stamp=this->now();
          m.ns="survivors"; m.id=survivors_.size()-1; m.type=3; m.action=0;
          m.pose.position.x=world_x; m.pose.position.y=world_y; m.pose.position.z=0.9; m.pose.orientation.w=1.0;
          m.scale.x=0.5; m.scale.y=0.5; m.scale.z=1.7;
          m.color.r=1.0; m.color.g=0.5; m.color.b=0.0; m.color.a=0.9;
          ma.markers.push_back(m);

          visualization_msgs::msg::Marker txt;
          txt.header=m.header; txt.ns="survivor_id"; txt.id=m.id+100; txt.type=9; txt.action=0;
          txt.pose.position.x=world_x; txt.pose.position.y=world_y; txt.pose.position.z=2.0; txt.pose.orientation.w=1.0;
          txt.scale.z=0.4; txt.color.r=1; txt.color.g=1; txt.color.b=1; txt.color.a=1;
          txt.text="S"+std::to_string(survivors_.size())+" "+std::to_string(conf).substr(0,4);
          ma.markers.push_back(txt);

          RCLCPP_INFO(this->get_logger(), "Survivor %zu at %.2f,%.2f conf %.2f dist %.1f", survivors_.size(), world_x, world_y, conf, dist);
        }
      }
    }

    marker_pub_->publish(ma);
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
    annotated_pub_->publish(*out_msg);
  }

  void estimateWorld(cv::Rect box, int img_w, int img_h, double &wx, double &wy, double &dist) {
    double fx;
    if (has_info_) fx=info_.k[0];
    else { double fov=69*M_PI/180; fx=(img_w/2)/std::tan(fov/2); }

    double bbox_h=box.height;
    dist = (1.7*fx)/bbox_h;
    dist=std::max(0.5, std::min(dist,10.0));
    double cx=box.x+box.width/2;
    double bearing=(cx-img_w/2)/fx;
    double rx=pose_.pose.position.x, ry=pose_.pose.position.y;
    auto q=pose_.pose.orientation;
    double siny=2*(q.w*q.z+q.x*q.y);
    double cosy=1-2*(q.y*q.y+q.z*q.z);
    double yaw=std::atan2(siny,cosy);
    wx=rx+dist*std::cos(yaw)-bearing*dist*std::sin(yaw);
    wy=ry+dist*std::sin(yaw)+bearing*dist*std::cos(yaw);
  }

  double conf_thresh_;
  int max_surv_;
  bool has_model_=false, has_pose_=false, has_info_=false;
  cv::dnn::Net net_;
  geometry_msgs::msg::PoseStamped pose_;
  sensor_msgs::msg::CameraInfo info_;
  std::vector<std::pair<double,double>> survivors_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SurvivorDetector>());
  rclcpp::shutdown();
  return 0;
}
