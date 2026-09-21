#!/usr/bin/env python3
"""YOLOv8 person detection for the maze drone.

Subscribes to ``/camera/rgb/image_raw``, detects persons (class 0),
projects them to 3D using depth + VIO pose.

Publishes MarkerArray for visualization and PoseStamped for mapping.

Optimized for edge: supports YOLOv8n (nano) with TensorRT engine,
runs at 10-15 Hz on Jetson Orin Nano.

Ubuntu 22.04 + ROS 2 Humble compatible.

Environment notes (ARM64 VMs):
  - Requires ``numpy<2.0.0`` for ``python3-opencv`` compatibility.
    See ``requirements.txt`` / ``constraints.txt`` and
    ``person_detection.numpy_compat``.
  - Model weights are resolved via ``ament_index_python`` package-share
    lookup (``<share>/person_detection/models/``) with a cache fallback
    at ``~/.cache/nidar/models/``. Missing weights trigger an automatic
    download of ``yolov8n.pt`` instead of ``[Errno 2]``.
"""

import json
import math
import os

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

# NumPy 2.x breaks ROS 2 python3-opencv (AttributeError: _ARRAY_API).
# Check before importing cv2 so operators get an actionable message.
try:
    from person_detection.numpy_compat import check_numpy_compat

    check_numpy_compat()
except ImportError:
    pass

import cv2  # noqa: E402  (after numpy compat check)
import numpy as np  # noqa: E402,F401  (re-exported for downstream use)

try:
    from person_detection.model_utils import resolve_model_path
except ImportError:
    resolve_model_path = None  # type: ignore[assignment]

try:
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class YOLODetector(Node):
    """Detect persons with YOLOv8 and project them into the map frame."""

    def __init__(self) -> None:
        super().__init__('yolo_detector')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('use_tensorrt', False)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter(
            'save_detections', '/tmp/person_detections.json'
        )

        self.model_name = str(self.get_parameter('model').value)
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.use_trt = bool(self.get_parameter('use_tensorrt').value)
        self.device = str(self.get_parameter('device').value)
        self.save_path = str(self.get_parameter('save_detections').value)

        self.bridge = CvBridge()
        self.current_pose = None
        self.detections: list = []
        self.detection_id = 0
        self.model = None

        self._load_model()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/offboard/pose', self.pose_callback, 10
        )

        self.marker_pub = self.create_publisher(MarkerArray, '/person_markers', 10)
        self.detection_pub = self.create_publisher(
            PoseStamped, '/person_detections', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/yolo/annotated_image', 10
        )

        self.get_logger().info('YOLO Person Detector ready')

    def _load_model(self) -> None:
        """Load YOLO weights via package-share resolution + cache fallback."""
        if not YOLO_AVAILABLE:
            self.get_logger().warn(
                'Ultralytics not installed, using dummy detector'
            )
            self.model = None
            return

        try:
            if resolve_model_path is not None:
                model_path = resolve_model_path(
                    self.model_name,
                    package_name='person_detection',
                    logger=self.get_logger(),
                )
            else:
                # Fallback when model_utils is unavailable (source run).
                model_path = self._legacy_resolve(self.model_name)

            # Prefer a TensorRT engine next to the resolved weights.
            if self.use_trt:
                engine_path = model_path.replace('.pt', '.engine')
                if os.path.exists(engine_path):
                    model_path = engine_path
                    self.get_logger().info(
                        f'Using TensorRT engine: {engine_path}'
                    )

            self.model = YOLO(model_path)
            self.get_logger().info(
                f'YOLO model loaded: {model_path} on {self.device}'
            )
        except Exception as exc:  # noqa: BLE001 - keep node alive in dummy mode
            self.get_logger().error(
                f'Failed to load YOLO: {exc}, using dummy detector'
            )
            self.model = None

    def _legacy_resolve(self, model_param: str) -> str:
        """Minimal resolver when model_utils cannot be imported."""
        if os.path.isabs(model_param) and os.path.exists(model_param):
            return model_param
        if os.path.exists(model_param):
            return os.path.abspath(model_param)
        try:
            from ament_index_python.packages import get_package_share_directory

            share = get_package_share_directory('person_detection')
            candidate = os.path.join(
                share, 'models', os.path.basename(model_param)
            )
            if os.path.exists(candidate):
                return candidate
        except Exception:  # noqa: BLE001 - fall through to basename
            pass
        self.get_logger().warn(
            f'Model {model_param!r} not found locally; '
            'passing to ultralytics for auto-download.'
        )
        return os.path.basename(model_param) or 'yolov8n.pt'

    def pose_callback(self, msg: PoseStamped) -> None:
        """Store the latest VIO / offboard pose."""
        self.current_pose = msg

    def image_callback(self, msg: Image) -> None:
        """Run YOLO inference on each RGB frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'CV bridge error: {exc}')
            return

        if self.model is None:
            annotated = cv_image.copy()
            cv2.putText(
                annotated,
                'YOLO not loaded - dummy mode',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            self.publish_annotated(annotated, msg.header)
            return

        results = self.model(
            cv_image,
            conf=self.conf_thres,
            classes=[0],
            verbose=False,
            device=self.device,
        )

        annotated = cv_image.copy()
        marker_array = MarkerArray()

        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())

                cv2.rectangle(
                    annotated,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    annotated,
                    f'Person {conf:.2f}',
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

                if self.current_pose is not None:
                    self._publish_detection(
                        x1, y1, x2, y2, conf, cv_image, msg, marker_array
                    )

        self.marker_pub.publish(marker_array)
        self.publish_annotated(annotated, msg.header)

        if len(self.detections) % 10 == 0 and self.detections:
            self.save_detections()

    def _publish_detection(
        self, x1, y1, x2, y2, conf, cv_image, msg, marker_array
    ) -> None:
        """Project a 2D bbox into the map frame and publish it."""
        img_h, img_w = cv_image.shape[:2]
        cx_img = (x1 + x2) / 2
        fov = 1.2
        focal = img_w / (2 * math.tan(fov / 2))
        bearing_x = (cx_img - img_w / 2) / focal

        bbox_h = y2 - y1
        if bbox_h > 0:
            distance = (1.7 * focal) / bbox_h
            distance = max(1.0, min(distance, 10.0))
        else:
            distance = 3.0

        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y

        quat = self.current_pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        world_x = (
            robot_x + distance * math.cos(yaw)
            - bearing_x * distance * math.sin(yaw)
        )
        world_y = (
            robot_y + distance * math.sin(yaw)
            + bearing_x * distance * math.cos(yaw)
        )

        det_msg = PoseStamped()
        det_msg.header = msg.header
        det_msg.header.frame_id = 'map'
        det_msg.pose.position.x = float(world_x)
        det_msg.pose.position.y = float(world_y)
        det_msg.pose.position.z = 0.9
        det_msg.pose.orientation.w = 1.0
        self.detection_pub.publish(det_msg)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'persons'
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
        marker.lifetime.sec = 0
        marker_array.markers.append(marker)

        text = Marker()
        text.header = marker.header
        text.ns = 'person_text'
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
        text.text = f'Person {conf:.2f} {distance:.1f}m'
        marker_array.markers.append(text)

        self.detections.append(
            {
                'x': float(world_x),
                'y': float(world_y),
                'conf': conf,
                'distance': float(distance),
                'timestamp': float(
                    self.get_clock().now().nanoseconds / 1e9
                ),
            }
        )

    def publish_annotated(self, cv_image, header) -> None:
        """Publish the annotated debug image."""
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header = header
            self.annotated_pub.publish(img_msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Publish annotated error: {exc}')

    def save_detections(self) -> None:
        """Persist detections to JSON for scoring / debugging."""
        try:
            with open(self.save_path, 'w', encoding='utf-8') as handle:
                json.dump(self.detections, handle, indent=2)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to save detections: {exc}')


def main(args=None) -> None:
    """Spin the YOLO detector node."""
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
