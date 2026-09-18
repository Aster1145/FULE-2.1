#!/usr/bin/env python3
"""
map_generator.py - Generates final 2D map for NIDAR AirMouse scoring
Requirements: Generate and display a 2D map of explored area

Input:
- /map (OccupancyGrid) from mapping_node / slam_toolbox
- /scan_3d_world (PointCloud2) from nodding lidar
- /survivor_markers

Output:
- /tmp/nidar_2d_map.pgm + .yaml (ROS map format)
- /tmp/nidar_2d_map_with_survivors.png (visual for judges)
- Publishes /final_2d_map for ground station

NIDAR scoring: Map quality, coverage, survivor tags
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
import numpy as np
import cv2
import yaml
import os

class MapGenerator(Node):
    def __init__(self):
        super().__init__('map_generator')
        self.map = None
        self.map_info = None
        self.survivors = []

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.survivor_sub = self.create_subscription(MarkerArray, '/survivor_markers', self.survivor_callback, 10)

        self.timer = self.create_timer(5.0, self.generate)

        self.get_logger().info("NIDAR Map Generator ready")

    def map_callback(self, msg):
        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def survivor_callback(self, msg):
        self.survivors = []
        for m in msg.markers:
            if m.ns == "survivors":
                self.survivors.append((m.pose.position.x, m.pose.position.y, m.id))

    def generate(self):
        if self.map is None:
            return

        # Create visual map
        # ROS map: -1 unknown, 0 free, 100 occupied
        # Image: unknown gray 128, free white 255, occupied black 0
        img = np.zeros((self.map.shape[0], self.map.shape[1], 3), dtype=np.uint8)
        img[self.map == -1] = [128, 128, 128]
        img[self.map == 0] = [255, 255, 255]
        img[self.map == 100] = [0, 0, 0]

        # Draw survivors as orange circles with ID
        if self.map_info is not None:
            for x, y, sid in self.survivors:
                gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
                gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
                if 0 <= gx < img.shape[1] and 0 <= gy < img.shape[0]:
                    cv2.circle(img, (gx, gy), 8, (0, 165, 255), -1)  # orange BGR
                    cv2.putText(img, f"S{sid+1}", (gx+10, gy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

            # Draw launch pad at home (0,0) - 2ft x 2ft = 0.61m
            home_gx = int((0 - self.map_info.origin.position.x) / self.map_info.resolution)
            home_gy = int((0 - self.map_info.origin.position.y) / self.map_info.resolution)
            size_px = int(0.61 / self.map_info.resolution)
            cv2.rectangle(img, (home_gx-size_px//2, home_gy-size_px//2), (home_gx+size_px//2, home_gy+size_px//2), (0,255,0), 2)
            cv2.putText(img, "HOME", (home_gx-20, home_gy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # Flip for correct orientation (ROS origin bottom-left, image top-left)
        img_flipped = cv2.flip(img, 0)

        # Save PNG for judges
        cv2.imwrite('/tmp/nidar_2d_map_with_survivors.png', img_flipped)
        cv2.imwrite('/tmp/nidar_2d_map.png', img_flipped)

        # Save PGM + YAML for ROS nav
        pgm_path = '/tmp/nidar_2d_map.pgm'
        yaml_path = '/tmp/nidar_2d_map.yaml'

        # PGM: grayscale
        gray = cv2.cvtColor(img_flipped, cv2.COLOR_BGR2GRAY)
        # Map values: free 254, occupied 0, unknown 205
        pgm_data = np.zeros_like(gray, dtype=np.uint8)
        pgm_data[self.map == 0] = 254
        pgm_data[self.map == 100] = 0
        pgm_data[self.map == -1] = 205
        pgm_data = np.flipud(pgm_data)  # PGM origin bottom-left

        cv2.imwrite(pgm_path, pgm_data)

        # YAML
        yaml_data = {
            'image': pgm_path,
            'resolution': float(self.map_info.resolution),
            'origin': [float(self.map_info.origin.position.x), float(self.map_info.origin.position.y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f)

        self.get_logger().info(f"NIDAR 2D map saved: {pgm_path}, {yaml_path}, PNG with {len(self.survivors)} survivors")

def main(args=None):
    rclpy.init(args=args)
    node = MapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
