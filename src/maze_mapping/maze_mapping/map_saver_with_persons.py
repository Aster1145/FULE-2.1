#!/usr/bin/env python3
"""Save the 2D map with a person overlay as an image.

Requires ``numpy<2.0.0`` for ``python3-opencv`` compatibility on ARM64
(see ``requirements.txt``).
"""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

try:
    from person_detection.numpy_compat import check_numpy_compat

    check_numpy_compat()
except ImportError:
    try:
        from air_mouse_mission.numpy_compat import check_numpy_compat

        check_numpy_compat()
    except ImportError:
        pass

import cv2  # noqa: E402  (after numpy compat check)
import numpy as np  # noqa: E402


class MapSaver(Node):
    """Persist the occupancy grid + person overlay to /tmp."""

    def __init__(self) -> None:
        super().__init__('map_saver')
        self.map = None
        self.map_info = None
        self.persons: list = []

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )
        self.person_sub = self.create_subscription(
            MarkerArray, '/person_markers', self.person_callback, 10
        )

        self.timer = self.create_timer(5.0, self.save_map)

    def map_callback(self, msg: OccupancyGrid) -> None:
        """Store the latest occupancy grid."""
        self.map = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info

    def person_callback(self, msg: MarkerArray) -> None:
        """Store person markers for overlay."""
        self.persons = []
        for marker in msg.markers:
            if marker.ns == 'persons':
                self.persons.append(
                    (marker.pose.position.x, marker.pose.position.y)
                )

    def save_map(self) -> None:
        """Render and save the map overlay image."""
        if self.map is None:
            return
        img = np.zeros(
            (self.map.shape[0], self.map.shape[1], 3), dtype=np.uint8
        )
        img[self.map == -1] = [128, 128, 128]
        img[self.map == 0] = [255, 255, 255]
        img[self.map == 100] = [0, 0, 0]

        if self.map_info is not None:
            for px, py in self.persons:
                gx = int(
                    (px - self.map_info.origin.position.x)
                    / self.map_info.resolution
                )
                gy = int(
                    (py - self.map_info.origin.position.y)
                    / self.map_info.resolution
                )
                if 0 <= gx < img.shape[1] and 0 <= gy < img.shape[0]:
                    cv2.circle(img, (gx, gy), 5, (255, 0, 0), -1)

        img_flipped = cv2.flip(img, 0)
        cv2.imwrite('/tmp/maze_map_with_persons.png', img_flipped)
        self.get_logger().info('Saved map to /tmp/maze_map_with_persons.png')


def main(args=None) -> None:
    """Spin the map saver node."""
    rclpy.init(args=args)
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
