#!/usr/bin/env python3
"""
map_saver_with_persons.py - Saves 2D map + person overlay as image
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
import numpy as np
import cv2
import json
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.map = None
        self.map_info = None
        self.persons = []

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.person_sub = self.create_subscription(MarkerArray, '/person_markers', self.person_callback, 10)

        self.timer = self.create_timer(5.0, self.save_map)

    def map_callback(self, msg):
        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def person_callback(self, msg):
        self.persons = []
        for m in msg.markers:
            if m.ns == "persons":
                self.persons.append((m.pose.position.x, m.pose.position.y))

    def save_map(self):
        if self.map is None:
            return
        # Create image: unknown gray, free white, occupied black
        img = np.zeros((self.map.shape[0], self.map.shape[1], 3), dtype=np.uint8)
        img[self.map == -1] = [128,128,128]
        img[self.map == 0] = [255,255,255]
        img[self.map == 100] = [0,0,0]

        # Draw persons as blue dots
        if self.map_info is not None:
            for px, py in self.persons:
                gx = int((px - self.map_info.origin.position.x) / self.map_info.resolution)
                gy = int((py - self.map_info.origin.position.y) / self.map_info.resolution)
                if 0 <= gx < img.shape[1] and 0 <= gy < img.shape[0]:
                    cv2.circle(img, (gx, gy), 5, (255,0,0), -1)

        # Flip vertically for correct orientation
        img_flipped = cv2.flip(img, 0)
        cv2.imwrite('/tmp/maze_map_with_persons.png', img_flipped)
        self.get_logger().info("Saved map to /tmp/maze_map_with_persons.png")

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
