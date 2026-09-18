#!/usr/bin/env python3
"""
offboard_controller.py - PX4 offboard control via DDS (px4_msgs)
Compatible with Ubuntu 22.04 + ROS2 Humble + PX4 1.14+

Subscribes to /trajectory (Path) from FUEL planner, converts to TrajectorySetpoint
Handles arming, takeoff, offboard mode switching, landing.

Frame conversions:
- ROS map: ENU (x east, y north, z up)
- PX4: NED (x north, y east, z down) and FRD body
We convert accordingly.

Usage:
ros2 run offboard_control offboard_controller --ros-args -p use_vio:=true
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint, VehicleCommand,
    VehicleLocalPosition, VehicleStatus, VehicleOdometry
)
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import time

class OffboardController(Node):
    def __init__(self):
        super().__init__('offboard_controller')
        self.declare_parameter('use_vio', True)
        self.declare_parameter('takeoff_height', -1.5)  # NED, negative up
        self.declare_parameter('max_speed', 1.0)

        self.use_vio = self.get_parameter('use_vio').value
        self.takeoff_h = self.get_parameter('takeoff_height').value
        self.max_speed = self.get_parameter('max_speed').value

        # PX4 QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/offboard/pose', 10)

        # Subscribers
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos_profile)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)
        self.traj_sub = self.create_subscription(Path, '/trajectory', self.trajectory_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/exploration/goal', self.goal_callback, 10)

        # State
        self.local_pos = None
        self.status = None
        self.current_traj = None
        self.traj_index = 0
        self.offboard_setpoint_counter = 0
        self.state = "IDLE"  # IDLE, ARMING, TAKEOFF, EXPLORING, LANDING
        self.arm_time = None

        # Timer 20Hz for offboard
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Offboard Controller initialized - Waiting for PX4")

    def local_pos_callback(self, msg):
        self.local_pos = msg
        # Publish pose in map frame (ENU) for mapping
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        # Convert NED to ENU: x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned
        pose.pose.position.x = msg.y  # NED y -> ENU x (east)
        pose.pose.position.y = msg.x  # NED x -> ENU y (north)
        pose.pose.position.z = -msg.z
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    def status_callback(self, msg):
        self.status = msg

    def trajectory_callback(self, msg):
        if len(msg.poses) == 0:
            return
        self.current_traj = msg.poses
        self.traj_index = 0
        self.get_logger().info(f"Received trajectory with {len(msg.poses)} waypoints")
        if self.state == "TAKEOFF":
            self.state = "EXPLORING"

    def goal_callback(self, msg):
        # Convert single goal to trajectory if no trajectory active
        if self.current_traj is None or self.traj_index >= len(self.current_traj):
            # Create simple path with one point
            path = Path()
            path.header = msg.header
            path.poses = [msg]
            self.trajectory_callback(path)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_traj_setpoint(self, x_ned, y_ned, z_ned, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x_ned, y_ned, z_ned]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_setpoint_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.send_vehicle_command(400, 1.0, 0.0)  # VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.get_logger().info("Arming command sent")

    def takeoff(self):
        # PX4 takeoff command not used in offboard, we just publish setpoint above
        pass

    def timer_callback(self):
        self.publish_offboard_mode()

        if self.local_pos is None:
            # Need to publish setpoint before offboard
            self.publish_traj_setpoint(0.0, 0.0, self.takeoff_h)
            self.offboard_setpoint_counter += 1
            return

        # State machine
        if self.state == "IDLE":
            # Publish 10 setpoints before switching to offboard
            if self.offboard_setpoint_counter < 20:
                self.publish_traj_setpoint(0.0, 0.0, self.takeoff_h)
                self.offboard_setpoint_counter += 1
            else:
                # Switch to offboard mode
                self.send_vehicle_command(176, 1.0, 6.0)  # DO_SET_MODE, custom mode 6 = offboard
                self.get_logger().info("Offboard mode command sent")
                self.state = "ARMING"
                self.arm_time = time.time()

        elif self.state == "ARMING":
            self.publish_traj_setpoint(0.0, 0.0, self.takeoff_h)
            if time.time() - self.arm_time > 1.0:
                self.arm()
                self.state = "TAKEOFF"
                self.takeoff_start = time.time()
                self.get_logger().info("ARMING -> TAKEOFF")

        elif self.state == "TAKEOFF":
            self.publish_traj_setpoint(0.0, 0.0, self.takeoff_h)
            # Check if reached takeoff height
            if abs(self.local_pos.z - self.takeoff_h) < 0.3:
                if time.time() - self.takeoff_start > 3.0:
                    self.get_logger().info("Takeoff complete, waiting for exploration goal")
                    # Stay hovering until trajectory arrives

        elif self.state == "EXPLORING":
            if self.current_traj is None or self.traj_index >= len(self.current_traj):
                # Hover at current position
                self.publish_traj_setpoint(self.local_pos.x, self.local_pos.y, self.takeoff_h)
            else:
                # Follow trajectory
                target = self.current_traj[self.traj_index]
                # Convert ENU to NED
                # target in ENU (map): x east, y north
                # NED: x north = y_enu, y east = x_enu, z down = -z_enu
                x_ned = target.pose.position.y
                y_ned = target.pose.position.x
                z_ned = -target.pose.position.z
                # Clamp z
                z_ned = max(z_ned, self.takeoff_h - 0.5)
                z_ned = min(z_ned, -0.5)

                self.publish_traj_setpoint(x_ned, y_ned, z_ned)

                # Check if reached waypoint
                dx = self.local_pos.x - x_ned
                dy = self.local_pos.y - y_ned
                dist = math.hypot(dx, dy)
                if dist < 0.4:
                    self.traj_index += 1
                    if self.traj_index >= len(self.current_traj):
                        self.get_logger().info("Trajectory complete")

        elif self.state == "LANDING":
            self.send_vehicle_command(185, 0.0, 0.0)  # LAND

def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
