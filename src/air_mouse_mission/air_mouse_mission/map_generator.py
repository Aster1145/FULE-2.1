#!/usr/bin/env python3
"""Generate the final 2D map for NIDAR AirMouse scoring.

Requirements: generate and display a 2D map of the explored area.

Input:
  - ``/map`` (OccupancyGrid) from mapping_node / slam_toolbox.
  - ``/scan_3d_world`` (PointCloud2) from nodding lidar.
  - ``/survivor_markers``.

Output:
  - ``/tmp/nidar_2d_map.pgm`` + ``.yaml`` (ROS map format).
  - ``/tmp/nidar_2d_map_with_survivors.png`` (visual for judges).

Environment notes (ARM64 VMs):
  - Requires ``numpy<2.0.0`` for ``python3-opencv`` compatibility
    (see ``requirements.txt`` and ``numpy_compat``).
"""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

try:
    from air_mouse_mission.numpy_compat import check_numpy_compat

    check_numpy_compat()
except ImportError:
    pass

import cv2  # noqa: E402  (after numpy compat check)
import numpy as np  # noqa: E402
import yaml  # noqa: E402


class MapGenerator(Node):
    """Render and persist the NIDAR 2D scoring map."""

    def __init__(self) -> None:
        super().__init__('map_generator')
        self.map = None
        self.map_info = None
        self.survivors: list = []

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.survivor_sub = self.create_subscription(
            MarkerArray, '/survivor_markers', self.survivor_callback, 10
        )

        self.timer = self.create_timer(5.0, self.generate)

        self.get_logger().info('NIDAR Map Generator ready')

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Store the latest occupancy grid."""
        self.map = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info

    def survivor_callback(self, msg: MarkerArray) -> None:
        """Store survivor markers for map overlay."""
        self.survivors = []
        for marker in msg.markers:
            if marker.ns == 'survivors':
                self.survivors.append(
                    (
                        marker.pose.position.x,
                        marker.pose.position.y,
                        marker.id,
                    )
                )

    def generate(self) -> None:
        """Render PNG + PGM/YAML outputs from the current map."""
        if self.map is None or self.map_info is None:
            return

        img = np.zeros(
            (self.map.shape[0], self.map.shape[1], 3), dtype=np.uint8
        )
        img[self.map == -1] = [128, 128, 128]
        img[self.map == 0] = [255, 255, 255]
        img[self.map == 100] = [0, 0, 0]

        for x, y, sid in self.survivors:
            gx = int(
                (x - self.map_info.origin.position.x)
                / self.map_info.resolution
            )
            gy = int(
                (y - self.map_info.origin.position.y)
                / self.map_info.resolution
            )
            if 0 <= gx < img.shape[1] and 0 <= gy < img.shape[0]:
                cv2.circle(img, (gx, gy), 8, (0, 165, 255), -1)
                cv2.putText(
                    img,
                    f'S{sid + 1}',
                    (gx + 10, gy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2,
                )

        home_gx = int(
            (0 - self.map_info.origin.position.x) / self.map_info.resolution
        )
        home_gy = int(
            (0 - self.map_info.origin.position.y) / self.map_info.resolution
        )
        size_px = int(0.61 / self.map_info.resolution)
        cv2.rectangle(
            img,
            (home_gx - size_px // 2, home_gy - size_px // 2),
            (home_gx + size_px // 2, home_gy + size_px // 2),
            (0, 255, 0),
            2,
        )
        cv2.putText(
            img,
            'HOME',
            (home_gx - 20, home_gy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

        img_flipped = cv2.flip(img, 0)

        cv2.imwrite('/tmp/nidar_2d_map_with_survivors.png', img_flipped)
        cv2.imwrite('/tmp/nidar_2d_map.png', img_flipped)

        pgm_path = '/tmp/nidar_2d_map.pgm'
        yaml_path = '/tmp/nidar_2d_map.yaml'

        gray = cv2.cvtColor(img_flipped, cv2.COLOR_BGR2GRAY)
        pgm_data = np.zeros_like(gray, dtype=np.uint8)
        pgm_data[self.map == 0] = 254
        pgm_data[self.map == 100] = 0
        pgm_data[self.map == -1] = 205
        pgm_data = np.flipud(pgm_data)

        cv2.imwrite(pgm_path, pgm_data)

        yaml_data = {
            'image': pgm_path,
            'resolution': float(self.map_info.resolution),
            'origin': [
                float(self.map_info.origin.position.x),
                float(self.map_info.origin.position.y),
                0.0,
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }
        with open(yaml_path, 'w', encoding='utf-8') as handle:
            yaml.dump(yaml_data, handle)

        self.get_logger().info(
            f'NIDAR 2D map saved: {pgm_path}, {yaml_path}, '
            f'PNG with {len(self.survivors)} survivors'
        )


def main(args=None) -> None:
    """Spin the map generator node."""
    rclpy.init(args=args)
    node = MapGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
