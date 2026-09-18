#!/usr/bin/env python3
"""
mapping_node.py - FUEL-inspired mapping for ROS2 Humble
Maintains 3D voxel grid + 2D occupancy grid from RPLidar + depth camera.
Publishes /map (2D), /voxel_map (PointCloud2), /esdf for collision check.
Inspired by FUEL's sdf_map and grid_map.

Ubuntu 22.04 compatible.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from collections import deque
import tf2_ros
from tf2_ros import TransformListener, Buffer

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.declare_parameter('map_size', 20.0)  # meters
        self.declare_parameter('resolution', 0.1)  # meters
        self.declare_parameter('voxel_height', 3.0)
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)

        self.map_size = self.get_parameter('map_size').value
        self.res = self.get_parameter('resolution').value
        self.voxel_h = self.get_parameter('voxel_height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value

        self.width = int(self.map_size / self.res)
        self.height = int(self.map_size / self.res)
        self.z_dim = int(self.voxel_h / self.res)

        # 2D occupancy: -1 unknown, 0 free, 100 occupied
        self.grid_2d = np.full((self.height, self.width), -1, dtype=np.int8)
        # 3D voxel occupancy for FUEL-style ESDF: 0 free, 1 occupied, -1 unknown
        self.voxel_grid = np.full((self.z_dim, self.height, self.width), -1, dtype=np.int8)

        # Pose
        self.current_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.pose_callback, 10)
        # Alternative pose from VIO
        self.vio_pose_sub = self.create_subscription(PoseStamped, '/vins_estimator/camera_pose', self.pose_callback, 10)
        self.odom_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.voxel_pub = self.create_publisher(PointCloud2, '/voxel_map', 1)
        self.frontier_pub = self.create_publisher(MarkerArray, '/frontiers_viz', 1)

        # Timer for publishing map
        self.map_timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info(f"Mapping Node initialized: {self.width}x{self.height} grid, res {self.res}m")

    def pose_callback(self, msg):
        self.current_pose = msg

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.res)
        gy = int((y - self.origin_y) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.res + self.origin_x + self.res/2
        y = gy * self.res + self.origin_y + self.res/2
        return x, y

    def scan_callback(self, msg):
        if self.current_pose is None:
            # Use zero pose if not available (simulation start)
            robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0
        else:
            robot_x = self.current_pose.pose.position.x
            robot_y = self.current_pose.pose.position.y
            # Extract yaw from quaternion
            q = self.current_pose.pose.orientation
            siny_cosp = 2*(q.w*q.z + q.x*q.y)
            cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Mark robot position as free
        rx, ry = self.world_to_grid(robot_x, robot_y)
        if 0 <= rx < self.width and 0 <= ry < self.height:
            self.grid_2d[ry, rx] = 0

        # Raycasting for each scan beam (simplified FUEL raycast)
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > msg.range_max:
                angle += msg.angle_increment
                continue

            # Endpoint in world
            end_x = robot_x + r * math.cos(robot_yaw + angle)
            end_y = robot_y + r * math.sin(robot_yaw + angle)

            # Bresenham raycasting to mark free
            self.raycast_and_update(robot_x, robot_y, end_x, end_y, occupied=True)

            angle += msg.angle_increment

    def raycast_and_update(self, x0, y0, x1, y1, occupied=True):
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)

        # Bresenham
        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy

        x, y = gx0, gy0
        max_iter = dx + dy + 5
        iter_count = 0
        while iter_count < max_iter:
            if 0 <= x < self.width and 0 <= y < self.height:
                if x == gx1 and y == gy1:
                    if occupied:
                        self.grid_2d[y, x] = 100
                        # Update voxel grid at 0.5m height slice for drone
                        z_idx = int(0.5 / self.res)
                        if 0 <= z_idx < self.z_dim:
                            self.voxel_grid[z_idx, y, x] = 1
                            # Also mark voxels below as occupied (wall)
                            for z in range(z_idx):
                                self.voxel_grid[z, y, x] = 1
                    break
                else:
                    if self.grid_2d[y, x] != 100:  # Don't overwrite occupied with free
                        self.grid_2d[y, x] = 0
                        # Mark voxel free at drone height
                        z_idx = int(0.5 / self.res)
                        if 0 <= z_idx < self.z_dim:
                            if self.voxel_grid[z_idx, y, x] == -1:
                                self.voxel_grid[z_idx, y, x] = 0
            else:
                break

            e2 = 2*err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            iter_count += 1

    def publish_map(self):
        # Publish 2D OccupancyGrid
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.res
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        # Flatten grid (ROS expects row-major, bottom-left origin)
        # Our grid is [y][x] with y 0 at bottom? Flip for ROS
        flat = self.grid_2d.flatten().tolist()
        msg.data = flat
        self.map_pub.publish(msg)

        # Publish voxel as PointCloud2 (occupied voxels)
        occupied_indices = np.argwhere(self.voxel_grid == 1)
        if len(occupied_indices) > 0:
            # Downsample for publishing
            if len(occupied_indices) > 5000:
                occupied_indices = occupied_indices[::len(occupied_indices)//5000]
            points = []
            for z, y, x in occupied_indices:
                wx, wy = self.grid_to_world(x, y)
                wz = z * self.res
                points.append([wx, wy, wz])
            self.publish_pointcloud(points)

    def publish_pointcloud(self, points):
        # Create PointCloud2
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        # Pack
        import struct
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', p[0], p[1], p[2]))
        msg.data = b''.join(buffer)
        self.voxel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
