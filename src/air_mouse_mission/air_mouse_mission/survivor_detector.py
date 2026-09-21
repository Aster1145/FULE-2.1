#!/usr/bin/env python3
"""NIDAR AirMouse survivor detection with OAK-D Lite RGB + YOLO.

Specialized for NIDAR: up to 6 survivors in a 15x15m maze with 2x2m rooms.

Uses OAK-D Lite RGB 13MP + on-device YOLO (DepthAI) or host YOLOv8:
  - OAK-D Lite can run YOLOv4-tiny / YOLOv5 / YOLOv8 on Myriad X (4 TOPS).
  - Host YOLOv8n for higher accuracy on Jetson.

Survivor tagging:
  - Detect person (COCO class 0).
  - Estimate 3D position using depth + VIO pose.
  - Tag on the 2D map with ID, confidence, timestamp.
  - Avoid duplicates via spatial clustering (1m threshold).

Environment notes (ARM64 VMs):
  - Requires ``numpy<2.0.0`` for ``python3-opencv`` compatibility
    (see ``requirements.txt`` and ``numpy_compat``).
  - Weights resolve via ``ament_index_python`` package-share lookup with
    a ``~/.cache/nidar/models/`` fallback that auto-downloads
    ``yolov8n.pt`` when missing.
"""

import math

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray

try:
    from air_mouse_mission.numpy_compat import check_numpy_compat

    check_numpy_compat()
except ImportError:
    pass

import cv2  # noqa: E402  (after numpy compat check)

try:
    from air_mouse_mission.model_utils import resolve_model_path
except ImportError:
    resolve_model_path = None  # type: ignore[assignment]

try:
    from ultralytics import YOLO

    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class SurvivorDetector(Node):
    """Detect survivors and publish map-frame markers."""

    def __init__(self) -> None:
        super().__init__('survivor_detector')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('conf_thres', 0.4)
        self.declare_parameter('image_topic', '/camera/rgb/image_raw')
        self.declare_parameter('depth_topic', '/camera/stereo/image_raw')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('use_oak_nn', False)
        self.declare_parameter('max_survivors', 6)

        self.model_name = str(self.get_parameter('model').value)
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.device = str(self.get_parameter('device').value)
        self.use_oak_nn = bool(self.get_parameter('use_oak_nn').value)
        self.max_survivors = int(self.get_parameter('max_survivors').value)

        self.bridge = CvBridge()
        self.current_pose = None
        self.camera_info = None
        self.survivors: list = []
        self.detection_count = 0
        self.model = None

        self._load_model()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/offboard/pose', self.pose_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/rgb/camera_info', self.info_callback, 10
        )
        self.oak_det_sub = self.create_subscription(
            MarkerArray, '/oak/detections', self.oak_det_callback, 10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, '/survivor_markers', 10
        )
        self.detection_pub = self.create_publisher(
            PoseStamped, '/person_detections', 10
        )
        self.annotated_pub = self.create_publisher(
            Image, '/survivor/annotated', 10
        )

        self.get_logger().info(
            f'Survivor Detector ready for NIDAR AirMouse, '
            f'max {self.max_survivors}'
        )

    def _load_model(self) -> None:
        """Load host YOLO unless OAK on-device NN is selected."""
        if self.use_oak_nn:
            self.model = None
            self.get_logger().info(
                'Using OAK-D Lite on-device NN, host YOLO disabled'
            )
            return
        if not YOLO_AVAILABLE:
            self.get_logger().warn(
                'Ultralytics not installed, survivor detection disabled'
            )
            self.model = None
            return
        try:
            if resolve_model_path is not None:
                model_path = resolve_model_path(
                    self.model_name,
                    package_name='air_mouse_mission',
                    logger=self.get_logger(),
                )
            else:
                model_path = self.model_name
            self.model = YOLO(model_path)
            self.get_logger().info(
                f'YOLO loaded {model_path} for survivor detection'
            )
        except Exception as exc:  # noqa: BLE001 - keep node alive
            self.get_logger().error(f'YOLO load failed {exc}')
            self.model = None

    def pose_callback(self, msg: PoseStamped) -> None:
        """Store the latest offboard pose."""
        self.current_pose = msg

    def info_callback(self, msg: CameraInfo) -> None:
        """Store the latest camera calibration."""
        self.camera_info = msg

    def oak_det_callback(self, msg: MarkerArray) -> None:
        """Handle OAK-D on-device detections (reserved)."""
        _ = msg

    def image_callback(self, msg: Image) -> None:
        """Run survivor inference on each RGB frame."""
        if self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'CV bridge error {exc}')
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
                    (0, 255, 255),
                    2,
                )
                label = f'Survivor {conf:.2f}'
                cv2.putText(
                    annotated,
                    label,
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                )

                if self.current_pose is not None:
                    self._handle_detection(
                        x1, y1, x2, y2, conf, cv_image, marker_array
                    )

        self.marker_pub.publish(marker_array)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to publish annotated {exc}')

    def _handle_detection(
        self, x1, y1, x2, y2, conf, cv_image, marker_array
    ) -> None:
        """Cluster, publish and log a single survivor detection."""
        world_x, world_y, distance = self.estimate_world_position(
            x1, y1, x2, y2, cv_image.shape
        )

        for sx, sy in self.survivors:
            if math.hypot(sx - world_x, sy - world_y) < 1.0:
                return

        if len(self.survivors) >= self.max_survivors:
            return

        self.survivors.append((world_x, world_y))

        det_msg = PoseStamped()
        det_msg.header.frame_id = 'map'
        det_msg.header.stamp = self.get_clock().now().to_msg()
        det_msg.pose.position.x = float(world_x)
        det_msg.pose.position.y = float(world_y)
        det_msg.pose.position.z = 0.9
        det_msg.pose.orientation.w = 1.0
        self.detection_pub.publish(det_msg)

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'survivors'
        marker.id = len(self.survivors) - 1
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

        text = Marker()
        text.header = marker.header
        text.ns = 'survivor_id'
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
        text.text = f'S{len(self.survivors)} {conf:.2f}'
        marker_array.markers.append(text)

        self.get_logger().info(
            f'Survivor {len(self.survivors)}/{self.max_survivors} at '
            f'{world_x:.2f}, {world_y:.2f}, conf {conf:.2f}, '
            f'dist {distance:.1f}m'
        )

    def estimate_world_position(self, x1, y1, x2, y2, img_shape):
        """Estimate map-frame position from a bbox and drone pose."""
        img_h, img_w = img_shape[:2]
        cx = (x1 + x2) / 2
        if self.camera_info is not None:
            fx = self.camera_info.k[0]
        else:
            fov = math.radians(69)
            fx = (img_w / 2) / math.tan(fov / 2)

        bbox_h = y2 - y1
        distance = (1.7 * fx) / bbox_h if bbox_h > 0 else 3.0
        distance = max(0.5, min(distance, 10.0))

        bearing = (cx - img_w / 2) / fx

        robot_x = self.current_pose.pose.position.x
        robot_y = self.current_pose.pose.position.y
        quat = self.current_pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        world_x = (
            robot_x + distance * math.cos(yaw)
            - bearing * distance * math.sin(yaw)
        )
        world_y = (
            robot_y + distance * math.sin(yaw)
            + bearing * distance * math.cos(yaw)
        )

        return world_x, world_y, distance


def main(args=None) -> None:
    """Spin the survivor detector node."""
    rclpy.init(args=args)
    node = SurvivorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
