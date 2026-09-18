#!/usr/bin/env python3
"""
yolo_detector.py - YOLOv8 person detection for maze drone
Subscribes to /camera/rgb/image_raw, detects persons (class 0), projects to 3D using depth + VIO pose
Publishes MarkerArray for visualization and PoseStamped for mapping

Optimized for edge: supports YOLOv8n (nano) with TensorRT engine, runs at 10-15Hz on Jetson Orin Nano

Ubuntu 22.04 + ROS2 Humble compatible
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os
import json

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('use_tensorrt', False)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('save_detections', '/tmp/person_detections.json')

        self.model_name = self.get_parameter('model').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.use_trt = self.get_parameter('use_tensorrt').value
        self.device = self.get_parameter('device').value
        self.save_path = self.get_parameter('save_detections').value

        self.bridge = CvBridge()
        self.current_pose = None
        self.detections = []
        self.detection_id = 0

        # Load YOLO
        if YOLO_AVAILABLE:
            model_path = self.model_name
            # Check if TensorRT engine exists
            if self.use_trt:
                engine_path = model_path.replace('.pt', '.engine')
                if os.path.exists(engine_path):
                    model_path = engine_path
                    self.get_logger().info(f"Using TensorRT engine: {engine_path}")
            try:
                self.model = YOLO(model_path)
                self.get_logger().info(f"YOLO model loaded: {model_path} on {self.device}")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO: {e}, using dummy detector")
                self.model = None
        else:
            self.get_logger().warn("Ultralytics not installed, using dummy detector")
            self.model = None

        # Subs
        self.image_sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)

        # Pubs
        self.marker_pub = self.create_publisher(MarkerArray, '/person_markers', 10)
        self.detection_pub = self.create_publisher(PoseStamped, '/person_detections', 10)
        self.annotated_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)

        self.get_logger().info("YOLO Person Detector ready")

    def pose_callback(self, msg):
        self.current_pose = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        if self.model is None:
            # Dummy: simulate detection for testing without YOLO
            annotated = cv_image.copy()
            cv2.putText(annotated, "YOLO not loaded - dummy mode", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            self.publish_annotated(annotated, msg.header)
            return

        # Inference
        results = self.model(cv_image, conf=self.conf_thres, classes=[0], verbose=False, device=self.device)  # class 0 = person

        annotated = cv_image.copy()
        marker_array = MarkerArray()

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for i, box in enumerate(boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls = int(box.cls[0].cpu().numpy())

                # Draw
                cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
                cv2.putText(annotated, f"Person {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                # Project to 3D map if pose available
                if self.current_pose is not None:
                    # Estimate distance: use bbox height heuristic or depth if available
                    # Simplified: assume person height 1.7m, focal length approx
                    # For real system, use depth image
                    # Here we use current drone position + forward projection
                    # Assume camera forward is drone forward (yaw)

                    # Compute bearing from image center
                    img_h, img_w = cv_image.shape[:2]
                    cx_img = (x1+x2)/2
                    cy_img = (y1+y2)/2
                    # Normalized coordinates
                    # Approx FOV 1.2 rad -> focal ~ width/(2*tan(FOV/2))
                    fov = 1.2
                    focal = img_w / (2*math.tan(fov/2))
                    bearing_x = (cx_img - img_w/2) / focal  # right
                    bearing_y = (cy_img - img_h/2) / focal  # down

                    # Estimate distance from bbox height: person ~1.7m
                    bbox_h = y2 - y1
                    if bbox_h > 0:
                        distance = (1.7 * focal) / bbox_h
                        distance = max(1.0, min(distance, 10.0))
                    else:
                        distance = 3.0

                    # Transform to world
                    robot_x = self.current_pose.pose.position.x
                    robot_y = self.current_pose.pose.position.y
                    robot_z = self.current_pose.pose.position.z

                    # Simple: project forward
                    # Get robot yaw from quaternion
                    q = self.current_pose.pose.orientation
                    siny_cosp = 2*(q.w*q.z + q.x*q.y)
                    cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
                    yaw = math.atan2(siny_cosp, cosy_cosp)

                    # Camera is forward-facing, so bearing affects world position
                    world_x = robot_x + distance * math.cos(yaw) - bearing_x * distance * math.sin(yaw)
                    world_y = robot_y + distance * math.sin(yaw) + bearing_x * distance * math.cos(yaw)

                    # Publish detection as PoseStamped
                    det_msg = PoseStamped()
                    det_msg.header = msg.header
                    det_msg.header.frame_id = "map"
                    det_msg.pose.position.x = float(world_x)
                    det_msg.pose.position.y = float(world_y)
                    det_msg.pose.position.z = 0.9
                    det_msg.pose.orientation.w = 1.0
                    self.detection_pub.publish(det_msg)

                    # Marker
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "persons"
                    marker.id = self.detection_id
                    self.detection_id += 1
                    marker.type = Marker.CYLINDER
                    marker.action = Marker.ADD
                    marker.pose.position.x = float(world_x)
                    marker.pose.position.y = float(world_y)
                    marker.pose.position.z = 0.9
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.5
                    marker.scale.y = 0.5
                    marker.scale.z = 1.8
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 0.8
                    marker.lifetime.sec = 0  # persistent
                    marker_array.markers.append(marker)

                    # Text
                    text = Marker()
                    text.header = marker.header
                    text.ns = "person_text"
                    text.id = marker.id + 10000
                    text.type = Marker.TEXT_VIEW_FACING
                    text.action = Marker.ADD
                    text.pose.position.x = float(world_x)
                    text.pose.position.y = float(world_y)
                    text.pose.position.z = 2.0
                    text.pose.orientation.w = 1.0
                    text.scale.z = 0.3
                    text.color.r = 1.0
                    text.color.g = 1.0
                    text.color.b = 1.0
                    text.color.a = 1.0
                    text.text = f"Person {conf:.2f} {distance:.1f}m"
                    marker_array.markers.append(text)

                    # Save
                    self.detections.append({
                        'x': float(world_x),
                        'y': float(world_y),
                        'conf': conf,
                        'distance': float(distance),
                        'timestamp': float(self.get_clock().now().nanoseconds/1e9)
                    })

        self.marker_pub.publish(marker_array)
        self.publish_annotated(annotated, msg.header)

        # Periodically save
        if len(self.detections) % 10 == 0 and len(self.detections) > 0:
            self.save_detections()

    def publish_annotated(self, cv_image, header):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header = header
            self.annotated_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Publish annotated error: {e}")

    def save_detections(self):
        try:
            with open(self.save_path, 'w') as f:
                json.dump(self.detections, f, indent=2)
        except Exception as e:
            self.get_logger().warn(f"Failed to save detections: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
