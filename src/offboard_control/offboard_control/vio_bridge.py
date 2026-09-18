#!/usr/bin/env python3
"""
vio_bridge.py - Converts VINS-Fusion odometry (ENU) to PX4 VehicleVisualOdometry (NED)
Required for PX4 to use VIO as external vision.

VINS outputs: /vins_estimator/camera_pose (PoseStamped) in ENU
PX4 expects: /fmu/in/vehicle_visual_odometry (VehicleOdometry) in NED, with covariance

Also publishes TF map->base_link and odom.

For real drone: set EKF2_AID_MASK to use vision.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class VIOBridge(Node):
    def __init__(self):
        super().__init__('vio_bridge')
        self.declare_parameter('input_topic', '/vins_estimator/camera_pose')
        self.declare_parameter('output_topic', '/fmu/in/vehicle_visual_odometry')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.vio_sub = self.create_subscription(PoseStamped, input_topic, self.vio_callback, 10)
        # Alternative odom topic
        self.odom_sub = self.create_subscription(Odometry, '/vins_estimator/odometry', self.odom_callback, 10)

        self.visual_odom_pub = self.create_publisher(VehicleOdometry, output_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)

        self.last_pose = None
        self.get_logger().info(f"VIO Bridge: {input_topic} -> {output_topic}")

    def pose_to_ned(self, pose_enu):
        # ENU to NED conversion
        # ENU: x east, y north, z up
        # NED: x north, y east, z down
        x_enu = pose_enu.position.x
        y_enu = pose_enu.position.y
        z_enu = pose_enu.position.z

        x_ned = y_enu
        y_ned = x_enu
        z_ned = -z_enu

        # Quaternion ENU to NED: need to rotate
        # Simplified: ENU to NED is 90 deg yaw + flip
        # For accurate conversion, use rotation matrix
        q_enu = [pose_enu.orientation.x, pose_enu.orientation.y, pose_enu.orientation.z, pose_enu.orientation.w]
        r_enu = R.from_quat(q_enu)
        # ENU to NED rotation matrix
        # Transform: R_ned = R_enu_to_ned * R_enu
        # R_enu_to_ned = [[0,1,0],[1,0,0],[0,0,-1]]
        mat_enu_to_ned = np.array([[0,1,0],[1,0,0],[0,0,-1]])
        r_ned = R.from_matrix(mat_enu_to_ned @ r_enu.as_matrix())
        q_ned = r_ned.as_quat()  # x,y,z,w

        return (x_ned, y_ned, z_ned), q_ned

    def vio_callback(self, msg):
        # Convert to VehicleOdometry
        (x_ned, y_ned, z_ned), q_ned = self.pose_to_ned(msg.pose)

        odom_msg = VehicleOdometry()
        odom_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        odom_msg.timestamp_sample = odom_msg.timestamp
        odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odom_msg.position = [x_ned, y_ned, z_ned]
        odom_msg.q = [float(q_ned[3]), float(q_ned[0]), float(q_ned[1]), float(q_ned[2])]  # w,x,y,z for PX4
        odom_msg.velocity = [0.0, 0.0, 0.0]
        odom_msg.angular_velocity = [0.0, 0.0, 0.0]
        odom_msg.position_variance = [0.1, 0.1, 0.1]
        odom_msg.orientation_variance = [0.1, 0.1, 0.1]
        odom_msg.velocity_variance = [0.1, 0.1, 0.1]

        self.visual_odom_pub.publish(odom_msg)

        # Also publish as PoseStamped for mapping
        self.pose_pub.publish(msg)
        self.last_pose = msg

    def odom_callback(self, msg):
        # Similar conversion for Odometry
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.vio_callback(pose_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = VIOBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
