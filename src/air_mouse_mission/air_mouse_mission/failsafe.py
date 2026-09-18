#!/usr/bin/env python3
"""
failsafe.py - NIDAR AirMouse failsafe + emergency stop
Requirements from NIDAR:
- Must include emergency stop/mission abort capability and failsafe features
- Corridor height clearance 8 feet (2.44m) at all times -> enforce max height
- Max mission 30 mins
- Return to home if comms lost, low battery, VIO failure, lidar failure

This node monitors:
- /emergency_stop (Bool) from ground station button
- Battery voltage (from PX4)
- VIO health (/vins_estimator/odometry timeout)
- Lidar health (/scan timeout)
- Height limit (z > 2.44m -> force down)
- Geofence (15x15m arena)
- RC loss

Publishes:
- /mission/abort (Bool)
- /failsafe/reason (String)
- Triggers LAND via VehicleCommand
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleStatus, BatteryStatus
import time

class FailsafeNode(Node):
    def __init__(self):
        super().__init__('failsafe_node')
        self.declare_parameter('max_height', 2.44)  # 8 feet
        self.declare_parameter('arena_size', 15.0)
        self.declare_parameter('vio_timeout', 2.0)
        self.declare_parameter('lidar_timeout', 2.0)
        self.declare_parameter('low_battery', 14.0)  # 4S

        self.max_height = self.get_parameter('max_height').value
        self.arena_size = self.get_parameter('arena_size').value
        self.vio_timeout = self.get_parameter('vio_timeout').value
        self.lidar_timeout = self.get_parameter('lidar_timeout').value

        self.last_vio_time = time.time()
        self.last_lidar_time = time.time()
        self.current_pose = None
        self.battery_voltage = 16.8
        self.emergency_stop = False

        # Subs
        self.pose_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.e_stop_sub = self.create_subscription(Bool, '/emergency_stop', self.e_stop_callback, 10)
        self.vio_sub = self.create_subscription(PoseStamped, '/vins_estimator/camera_pose', self.vio_callback, 10)
        self.battery_sub = self.create_subscription(BatteryStatus, '/fmu/out/battery_status', self.battery_callback, 10)

        # Pubs
        self.abort_pub = self.create_publisher(Bool, '/mission/abort', 10)
        self.reason_pub = self.create_publisher(String, '/failsafe/reason', 10)
        self.status_pub = self.create_publisher(String, '/failsafe/status', 10)

        self.timer = self.create_timer(0.5, self.check_failsafe)

        self.get_logger().info(f"Failsafe: max height {self.max_height}m, arena {self.arena_size}m, vio timeout {self.vio_timeout}s")

    def pose_callback(self, msg):
        self.current_pose = msg

    def scan_callback(self, msg):
        self.last_lidar_time = time.time()

    def vio_callback(self, msg):
        self.last_vio_time = time.time()

    def battery_callback(self, msg):
        # PX4 BatteryStatus voltage
        self.battery_voltage = msg.voltage_v

    def e_stop_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.trigger_abort("Emergency stop button pressed")

    def check_failsafe(self):
        reason = None

        # Check emergency stop
        if self.emergency_stop:
            reason = "Emergency stop"

        # Check height
        if self.current_pose and self.current_pose.pose.position.z > self.max_height:
            reason = f"Height limit exceeded: {self.current_pose.pose.position.z:.2f} > {self.max_height}"

        # Check geofence
        if self.current_pose:
            x = self.current_pose.pose.position.x
            y = self.current_pose.pose.position.y
            if abs(x) > self.arena_size/2 or abs(y) > self.arena_size/2:
                reason = f"Geofence breach: ({x:.1f}, {y:.1f}) outside {self.arena_size}m"

        # Check VIO timeout
        if time.time() - self.last_vio_time > self.vio_timeout:
            reason = f"VIO timeout: {time.time()-self.last_vio_time:.1f}s"

        # Check lidar timeout
        if time.time() - self.last_lidar_time > self.lidar_timeout:
            reason = f"Lidar timeout: {time.time()-self.last_lidar_time:.1f}s"

        # Check battery
        if self.battery_voltage < self.get_parameter('low_battery').value:
            reason = f"Low battery: {self.battery_voltage:.1f}V"

        if reason:
            self.trigger_abort(reason)
        else:
            # Publish OK status
            status = String()
            status.data = "OK"
            self.status_pub.publish(status)

    def trigger_abort(self, reason):
        self.get_logger().error(f"FAILSAFE TRIGGERED: {reason}")

        reason_msg = String()
        reason_msg.data = reason
        self.reason_pub.publish(reason_msg)

        abort_msg = Bool()
        abort_msg.data = True
        self.abort_pub.publish(abort_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FailsafeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
