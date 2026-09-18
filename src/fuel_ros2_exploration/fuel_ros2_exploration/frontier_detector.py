#!/usr/bin/env python3
"""
frontier_detector.py - Frontier Information Structure (FIS) inspired by FUEL
Implements incremental frontier detection, clustering, and information gain evaluation.

FUEL paper: Frontier Information Structure maintains crucial info for exploration planning.
We port logic to ROS2 Python.

- Detects frontiers: free cell adjacent to unknown
- Clusters via BFS/DBSCAN
- Filters by size
- Computes information gain: count unknown voxels in sensor FoV from frontier viewpoint
- Publishes frontier goals sorted by gain / distance
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from collections import deque
from sklearn.cluster import DBSCAN

class FrontierDetector(Node):
    def __init__(self):
        super().__init__('frontier_detector')
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('cluster_tolerance', 0.5)  # meters
        self.declare_parameter('sensor_range', 4.0)
        self.declare_parameter('info_gain_threshold', 5)

        self.min_size = self.get_parameter('min_frontier_size').value
        self.cluster_tol = self.get_parameter('cluster_tolerance').value
        self.sensor_range = self.get_parameter('sensor_range').value

        self.map = None
        self.map_info = None
        self.current_pose = None

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)
        self.vio_pose_sub = self.create_subscription(PoseStamped, '/vins_estimator/camera_pose', self.pose_callback, 10)

        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/exploration/goal', 10)
        self.frontier_list_pub = self.create_publisher(MarkerArray, '/frontiers_list', 10)

        self.timer = self.create_timer(1.0, self.detect_frontiers)

        self.frontiers = []  # list of clusters: {'centroid': (x,y), 'points': [(x,y)], 'gain': float}
        self.get_logger().info("Frontier Detector (FIS) initialized")

    def pose_callback(self, msg):
        self.current_pose = msg

    def map_callback(self, msg):
        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def is_frontier_cell(self, y, x):
        # Must be free (0)
        if self.map[y, x] != 0:
            return False
        # Check 8 neighbors for unknown (-1)
        h, w = self.map.shape
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dy == 0 and dx == 0:
                    continue
                ny, nx = y+dy, x+dx
                if 0 <= ny < h and 0 <= nx < w:
                    if self.map[ny, nx] == -1:
                        return True
        return False

    def detect_frontiers(self):
        if self.map is None or self.map_info is None:
            return

        h, w = self.map.shape
        visited = np.zeros((h, w), dtype=bool)
        frontiers_raw = []

        # BFS to find all frontier cells (FUEL's incremental FIS)
        for y in range(1, h-1):
            for x in range(1, w-1):
                if visited[y, x]:
                    continue
                if self.is_frontier_cell(y, x):
                    # BFS cluster this frontier
                    cluster = []
                    queue = deque()
                    queue.append((y, x))
                    visited[y, x] = True
                    while queue:
                        cy, cx = queue.popleft()
                        cluster.append((cy, cx))
                        # 8-connected
                        for dy in [-1,0,1]:
                            for dx in [-1,0,1]:
                                ny, nx = cy+dy, cx+dx
                                if 0 <= ny < h and 0 <= nx < w and not visited[ny, nx]:
                                    visited[ny, nx] = True
                                    if self.is_frontier_cell(ny, nx):
                                        queue.append((ny, nx))
                    if len(cluster) >= self.min_size:
                        frontiers_raw.append(cluster)

        # Convert clusters to world coordinates and compute centroid + gain
        self.frontiers = []
        for cluster in frontiers_raw:
            points_world = []
            for y, x in cluster:
                wx = x * self.map_info.resolution + self.map_info.origin.position.x
                wy = y * self.map_info.resolution + self.map_info.origin.position.y
                points_world.append((wx, wy))
            # Centroid
            cx = np.mean([p[0] for p in points_world])
            cy = np.mean([p[1] for p in points_world])
            # Info gain: count unknown cells within sensor_range from centroid
            gain = self.compute_info_gain(cx, cy)
            self.frontiers.append({
                'centroid': (cx, cy),
                'points': points_world,
                'gain': gain,
                'size': len(cluster)
            })

        # Sort by gain / distance cost (FUEL's frontier cost)
        if self.current_pose is not None:
            rx = self.current_pose.pose.position.x
            ry = self.current_pose.pose.position.y
            for f in self.frontiers:
                dx = f['centroid'][0] - rx
                dy = f['centroid'][1] - ry
                dist = math.hypot(dx, dy)
                # FUEL-like cost: -gain + distance penalty
                f['cost'] = -f['gain'] + 0.5 * dist
            self.frontiers.sort(key=lambda f: f['cost'])
        else:
            self.frontiers.sort(key=lambda f: -f['gain'])

        self.publish_frontiers()
        self.publish_best_goal()

        if len(self.frontiers) > 0:
            self.get_logger().info(f"Found {len(self.frontiers)} frontiers, best gain {self.frontiers[0]['gain']:.1f} at {self.frontiers[0]['centroid']}")

    def compute_info_gain(self, cx, cy):
        if self.map is None:
            return 0
        # Count unknown cells within sensor_range circle
        res = self.map_info.resolution
        radius_cells = int(self.sensor_range / res)
        gx = int((cx - self.map_info.origin.position.x) / res)
        gy = int((cy - self.map_info.origin.position.y) / res)
        h, w = self.map.shape
        gain = 0
        for dy in range(-radius_cells, radius_cells+1):
            for dx in range(-radius_cells, radius_cells+1):
                if dx*dx + dy*dy > radius_cells*radius_cells:
                    continue
                ny, nx = gy+dy, gx+dx
                if 0 <= ny < h and 0 <= nx < w:
                    if self.map[ny, nx] == -1:
                        gain += 1
        return gain

    def publish_frontiers(self):
        marker_array = MarkerArray()
        for i, f in enumerate(self.frontiers):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "frontiers"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = f['centroid'][0]
            m.pose.position.y = f['centroid'][1]
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.r = 1.0 - min(f['gain']/50.0, 1.0)
            m.color.g = min(f['gain']/50.0, 1.0)
            m.color.b = 0.0
            m.color.a = 0.8
            marker_array.markers.append(m)

            # Text marker for gain
            mt = Marker()
            mt.header = m.header
            mt.ns = "frontier_gain"
            mt.id = i + 1000
            mt.type = Marker.TEXT_VIEW_FACING
            mt.action = Marker.ADD
            mt.pose.position.x = f['centroid'][0]
            mt.pose.position.y = f['centroid'][1]
            mt.pose.position.z = 1.0
            mt.pose.orientation.w = 1.0
            mt.scale.z = 0.3
            mt.color.r = 1.0
            mt.color.g = 1.0
            mt.color.b = 1.0
            mt.color.a = 1.0
            mt.text = f"G:{f['gain']:.0f}"
            marker_array.markers.append(mt)

        self.frontier_pub.publish(marker_array)

    def publish_best_goal(self):
        if len(self.frontiers) == 0:
            return
        best = self.frontiers[0]
        if best['gain'] < self.get_parameter('info_gain_threshold').value:
            return
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = best['centroid'][0]
        goal.pose.position.y = best['centroid'][1]
        goal.pose.position.z = 1.5  # fixed exploration height
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
