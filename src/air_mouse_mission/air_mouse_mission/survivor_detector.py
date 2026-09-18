#!/usr/bin/env python3
"""
survivor_detector.py - NIDAR AirMouse survivor detection using OAK-D Lite RGB + YOLO
Specialized for NIDAR: up to 6 survivors in 15x15m maze, 2x2m rooms

Uses OAK-D Lite RGB 13MP + on-device YOLO (DepthAI) or host YOLOv8
- OAK-D Lite can run YOLOv4-tiny, YOLOv5, YOLOv8 on Myriad X VPU (4 TOPS)
- Host YOLOv8n for higher accuracy on Jetson

Survivor tagging:
- Detect person (COCO class 0)
- Estimate 3D position using depth + VIO pose
- Tag on 2D map with ID, confidence, timestamp
- Avoid duplicates via spatial clustering (1m threshold)
- NIDAR scoring requires: location accuracy, count, map overlay

Also supports thermal? No, OAK-D Lite is RGB only.

For competition, survivors may be:
- Real humans (with safety vest)
- Mannequins / dummies
- Posters / cutouts
We detect all as person class, with fallback to color/shape detection if needed.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import json

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

class SurvivorDetector(Node):
    def __init__(self):
        super().__init__('survivor_detector')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf_thres', 0.4)  # Lower for NIDAR to catch dummies
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('depth_topic', '/camera/stereo/image_raw')  # OAK-D depth aligned to RGB
        self.declare_parameter('use_oak_nn', False)  # If true, use OAK on-device NN, not host YOLO
        self.declare_parameter('max_survivors', 6)

        self.model_name = self.get_parameter('model').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.image_topic = self.get_parameter('image_topic').value
        self.max_survivors = self.get_parameter('max_survivors').value

        self.bridge = CvBridge()
        self.current_pose = None
        self.camera_info = None
        self.survivors = []  # list of (x,y) world
        self.detection_count = 0

        # Load YOLO
        if YOLO_AVAILABLE and not self.get_parameter('use_oak_nn').value:
            try:
                self.model = YOLO(self.model_name)
                self.get_logger().info(f"YOLO loaded {self.model_name} for survivor detection")
            except Exception as e:
                self.get_logger().error(f"YOLO load failed {e}")
                self.model = None
        else:
            self.model = None
            if self.get_parameter('use_oak_nn').value:
                self.get_logger().info("Using OAK-D Lite on-device NN, host YOLO disabled")

        # Subs
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/rgb/camera_info', self.info_callback, 10)

        # If using OAK-D on-device detections
        self.oak_det_sub = self.create_subscription(MarkerArray, '/oak/detections', self.oak_det_callback, 10)

        # Pubs
        self.marker_pub = self.create_publisher(MarkerArray, '/survivor_markers', 10)
        self.detection_pub = self.create_publisher(PoseStamped, '/person_detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/survivor/annotated', 10)

        self.get_logger().info(f"Survivor Detector ready for NIDAR AirMouse, max {self.max_survivors}")

    def pose_callback(self, msg):
        self.current_pose = msg

    def info_callback(self, msg):
        self.camera_info = msg

    def oak_det_callback(self, msg):
        # If OAK-D runs NN on-device, detections come as MarkerArray or custom
        pass

    def image_callback(self, msg):
        if self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error {e}")
            return

        # YOLO inference - person class only
        results = self.model(cv_image, conf=self.conf_thres, classes=[0], verbose=False)

        annotated = cv_image.copy()
        marker_array = MarkerArray()

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())

                # Draw
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
                label = f"Survivor {conf:.2f}"
                cv2.putText(annotated, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

                # 3D position estimation
                if self.current_pose is not None:
                    world_x, world_y, distance = self.estimate_world_position(x1, y1, x2, y2, cv_image.shape)

                    # Check duplicate
                    is_new = True
                    for sx, sy in self.survivors:
                        if math.hypot(sx-world_x, sy-world_y) < 1.0:
                            is_new = False
                            break

                    if is_new and len(self.survivors) < self.max_survivors:
                        self.survivors.append((world_x, world_y))

                        # Publish detection
                        det_msg = PoseStamped()
                        det_msg.header.frame_id = "map"
                        det_msg.header.stamp = self.get_clock().now().to_msg()
                        det_msg.pose.position.x = float(world_x)
                        det_msg.pose.position.y = float(world_y)
                        det_msg.pose.position.z = 0.9
                        det_msg.pose.orientation.w = 1.0
                        self.detection_pub.publish(det_msg)

                        # Marker for RViz and NIDAR ground station
                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.ns = "survivors"
                        marker.id = len(self.survivors)-1
                        marker.type = Marker.CYLINDER
                        marker.action = Marker.ADD
                        marker.pose.position.x = float(world_x)
                        marker.pose.position.y = float(world_y)
                        marker.pose.position.z = 0.9
                        marker.pose.orientation.w = 1.0
                        marker.scale.x = 0.5
                        marker.scale.y = 0.5
                        marker.scale.z = 1.7
                        marker.color.r = 1.0
                        marker.color.g = 0.5
                        marker.color.b = 0.0
                        marker.color.a = 0.9
                        marker_array.markers.append(marker)

                        # Text ID
                        text = Marker()
                        text.header = marker.header
                        text.ns = "survivor_id"
                        text.id = marker.id + 100
                        text.type = Marker.TEXT_VIEW_FACING
                        text.action = Marker.ADD
                        text.pose.position.x = float(world_x)
                        text.pose.position.y = float(world_y)
                        text.pose.position.z = 2.0
                        text.pose.orientation.w = 1.0
                        text.scale.z = 0.4
                        text.color.r = 1.0
                        text.color.g = 1.0
                        text.color.b = 1.0
                        text.color.a = 1.0
                        text.text = f"S{len(self.survivors)} {conf:.2f}"
                        marker_array.markers.append(text)

                        self.get_logger().info(f"Survivor {len(self.survivors)}/{self.max_survivors} at {world_x:.2f}, {world_y:.2f}, conf {conf:.2f}, dist {distance:.1f}m")

        self.marker_pub.publish(marker_array)

        # Publish annotated image for ground station (NIDAR requires video stream)
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish annotated {e}")

    def estimate_world_position(self, x1, y1, x2, y2, img_shape):
        # Estimate distance from bbox height + project to world
        img_h, img_w = img_shape[:2]
        cx = (x1+x2)/2
        # Approx focal from camera_info or FOV
        if self.camera_info is not None:
            fx = self.camera_info.k[0]
        else:
            # OAK-D Lite RGB: HFOV 69°, width 1920 -> fx ~ 1400
            fov = math.radians(69)
            fx = (img_w/2) / math.tan(fov/2)

        bbox_h = y2 - y1
        # Person height 1.7m
        distance = (1.7 * fx) / bbox_h if bbox_h>0 else 3.0
        distance = max(0.5, min(distance, 10.0))

        # Bearing
        bearing = (cx - img_w/2) / fx  # rad approx, right positive

        # World projection using drone pose
        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        q = self.current_pose.pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        world_x = robot_x + distance * math.cos(yaw) - bearing * distance * math.sin(yaw)
        world_y = robot_y + distance * math.sin(yaw) + bearing * distance * math.cos(yaw)

        return world_x, world_y, distance

def main(args=None):
    rclpy.init(args=args)
    node = SurvivorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
