#!/usr/bin/env python3
"""
scan_to_3d.py - Converts 2D RPLidar scans + nodding angle into 3D pointcloud
Core for NIDAR AirMouse: 2D->3D via oscillation

Input:
- /scan (LaserScan) from RPLidar, 360° horizontal
- /servo_angle / /joint_states (pitch angle of lidar mount)
- /offboard/pose or /camera/odom (drone pose for world frame)

Output:
- /scan_3d (PointCloud2) - accumulated 3D points in lidar frame + world frame
- /map_3d (OctoMap style) for FUEL planner
- /scan (still publish 2D projection for 2D mapping)

Algorithm (nodding lidar):
For each 2D scan point at (r, theta) in lidar's local horizontal plane,
with mount pitch = phi (servo angle):
- Point in mount frame: x = r*cos(theta), y = r*sin(theta), z=0
- Rotate by pitch phi around Y axis (nodding): 
  x' = x*cos(phi) + z*sin(phi)
  y' = y
  z' = -x*sin(phi) + z*cos(phi)
- Then transform by drone pose to world frame

Accumulate over full nodding sweep (e.g., -45° to +45°) to get dense 3D scan of room.
For 15x15m maze, 8ft height, this captures floor, walls, ceiling, survivors.

Also generates 2D projection for 2D map (required for NIDAR scoring).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np
import math
import struct
from collections import deque

class ScanTo3D(Node):
    def __init__(self):
        super().__init__('scan_to_3d')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('servo_angle_topic', '/servo_angle_cmd')
        self.declare_parameter('accumulate_scans', 20)  # number of 2D scans per 3D cloud
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('publish_2d_projection', True)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.servo_topic = self.get_parameter('servo_angle_topic').value
        self.accumulate_n = self.get_parameter('accumulate_scans').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value

        self.current_pitch = 0.0
        self.current_pose = None
        self.scan_buffer = deque(maxlen=self.accumulate_n)

        # Subs
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.pitch_sub = self.create_subscription(Float64, self.servo_topic, self.pitch_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)

        # Pubs
        self.cloud_pub = self.create_publisher(PointCloud2, '/scan_3d', 10)
        self.cloud_world_pub = self.create_publisher(PointCloud2, '/scan_3d_world', 10)
        self.scan_2d_pub = self.create_publisher(LaserScan, '/scan_2d_projected', 10)

        self.get_logger().info(f"ScanTo3D: {self.accumulate_n} scans per 3D cloud, pitch from {self.servo_topic}")

    def pitch_callback(self, msg):
        self.current_pitch = msg.data

    def joint_callback(self, msg):
        try:
            idx = msg.name.index('rplidar_pitch_joint')
            self.current_pitch = msg.position[idx]
        except ValueError:
            pass

    def pose_callback(self, msg):
        self.current_pose = msg

    def scan_callback(self, msg):
        # Store scan with current pitch
        self.scan_buffer.append({
            'scan': msg,
            'pitch': self.current_pitch,
            'pose': self.current_pose
        })

        # When buffer full, generate 3D cloud
        if len(self.scan_buffer) >= self.accumulate_n:
            self.generate_3d_cloud()
            # Optionally clear buffer for next sweep
            # self.scan_buffer.clear()  # Keep sliding window

    def generate_3d_cloud(self):
        points_local = []  # in base_link frame
        points_world = []

        for entry in self.scan_buffer:
            scan = entry['scan']
            pitch = entry['pitch']
            pose = entry['pose']

            # Precompute pitch rotation
            cos_p = math.cos(pitch)
            sin_p = math.sin(pitch)

            angle = scan.angle_min
            for r in scan.ranges:
                if math.isinf(r) or math.isnan(r) or r < self.min_range or r > self.max_range:
                    angle += scan.angle_increment
                    continue

                # 2D point in lidar horizontal plane (lidar frame)
                # RPLidar: 0 angle = forward, CCW
                x_lidar = r * math.cos(angle)
                y_lidar = r * math.sin(angle)
                z_lidar = 0.0

                # Rotate by pitch (nodding) around Y axis
                # Mount pitch rotates lidar forward up/down
                # x' = x*cos(pitch) + z*sin(pitch)
                # z' = -x*sin(pitch) + z*cos(pitch)
                x_mount = x_lidar * cos_p + z_lidar * sin_p
                y_mount = y_lidar
                z_mount = -x_lidar * sin_p + z_lidar * cos_p

                # Transform to base_link: assume lidar mounted at (0,0,0.12) + pitch joint
                # For simplicity, add mount offset
                x_base = x_mount
                y_base = y_mount
                z_base = z_mount + 0.12  # lidar height

                points_local.append([x_base, y_base, z_base])

                # Transform to world if pose available
                if pose is not None:
                    # Simple ENU transform: rotate by yaw and translate
                    q = pose.pose.orientation
                    siny_cosp = 2*(q.w*q.z + q.x*q.y)
                    cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
                    yaw = math.atan2(siny_cosp, cosy_cosp)

                    cos_y = math.cos(yaw)
                    sin_y = math.sin(yaw)

                    x_world = pose.pose.position.x + x_base * cos_y - y_base * sin_y
                    y_world = pose.pose.position.y + x_base * sin_y + y_base * cos_y
                    z_world = pose.pose.position.z + z_base

                    points_world.append([x_world, y_world, z_world])

                angle += scan.angle_increment

        # Publish local cloud
        if len(points_local) > 0:
            self.publish_cloud(points_local, self.cloud_pub, frame_id="base_link")

        # Publish world cloud (for mapping)
        if len(points_world) > 0:
            self.publish_cloud(points_world, self.cloud_world_pub, frame_id="map")

        self.get_logger().info(f"Generated 3D cloud: {len(points_local)} points from {len(self.scan_buffer)} scans, pitch range {min([e['pitch'] for e in self.scan_buffer]):.2f} to {max([e['pitch'] for e in self.scan_buffer]):.2f} rad")

    def publish_cloud(self, points, publisher, frame_id="map"):
        # Downsample if too many
        if len(points) > 10000:
            points = points[::len(points)//10000]

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
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
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', float(p[0]), float(p[1]), float(p[2])))
        msg.data = b''.join(buffer)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanTo3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
