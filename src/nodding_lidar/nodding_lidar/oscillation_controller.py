#!/usr/bin/env python3
"""
oscillation_controller.py - Controls servo that nods RPLidar to convert 2D to 3D
For NIDAR AirMouse: RPLidar A2/A3 mounted on servo (e.g., Dynamixel XL430 or MG996R via Arduino)

Mechanism:
- Servo oscillates pitch from -45° to +45° (or -30° to +60° to cover floor to ceiling)
- At each angle, 2D scan (360° horizontal) is captured
- Combined to form 3D pointcloud
- Frequency: 0.5-1 Hz nodding, 10Hz lidar = ~20 scans per sweep

Hardware options:
1. Arduino + Servo: Arduino subscribes to /servo_angle_cmd, publishes /servo_angle
2. Dynamixel SDK: Direct ROS2 control via dynamixel_sdk
3. PWM via PCA9685 on Jetson/RPi

This node publishes servo angle and controls oscillation.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
import math
import time

class OscillationController(Node):
    def __init__(self):
        super().__init__('oscillation_controller')
        self.declare_parameter('min_angle_deg', -45.0)  # min pitch
        self.declare_parameter('max_angle_deg', 45.0)   # max pitch
        self.declare_parameter('frequency_hz', 0.5)     # oscillation freq
        self.declare_parameter('mode', 'sinusoidal')    # sinusoidal or triangular
        self.declare_parameter('servo_topic', '/servo_angle_cmd')
        self.declare_parameter('enable', True)

        self.min_angle = math.radians(self.get_parameter('min_angle_deg').value)
        self.max_angle = math.radians(self.get_parameter('max_angle_deg').value)
        self.freq = self.get_parameter('frequency_hz').value
        self.mode = self.get_parameter('mode').value
        self.servo_topic = self.get_parameter('servo_topic').value

        # Publishers
        self.angle_pub = self.create_publisher(Float64, self.servo_topic, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.scan_enable_pub = self.create_publisher(Bool, '/nodding_scan_enable', 10)

        # State
        self.start_time = time.time()
        self.current_angle = 0.0

        # Timer 50Hz for smooth servo
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info(f"Nodding Lidar Controller: {self.get_parameter('min_angle_deg').value}° to {self.get_parameter('max_angle_deg').value}° @ {self.freq}Hz, mode {self.mode}")

    def update(self):
        elapsed = time.time() - self.start_time

        # Compute target angle
        if self.mode == 'sinusoidal':
            # Sinusoidal oscillation: smooth
            amplitude = (self.max_angle - self.min_angle) / 2
            center = (self.max_angle + self.min_angle) / 2
            self.current_angle = center + amplitude * math.sin(2 * math.pi * self.freq * elapsed)
        else:
            # Triangular (sawtooth): linear sweep
            period = 1.0 / self.freq
            phase = (elapsed % period) / period
            if phase < 0.5:
                # Up sweep
                self.current_angle = self.min_angle + (self.max_angle - self.min_angle) * (phase * 2)
            else:
                # Down sweep
                self.current_angle = self.max_angle - (self.max_angle - self.min_angle) * ((phase - 0.5) * 2)

        # Publish angle command
        msg = Float64()
        msg.data = self.current_angle
        self.angle_pub.publish(msg)

        # Publish joint state for TF and 3D reconstruction
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['rplidar_pitch_joint']
        joint_msg.position = [self.current_angle]
        joint_msg.velocity = [0.0]
        self.joint_pub.publish(joint_msg)

        # Enable scanning always (or disable at extremes to reduce distortion)
        enable_msg = Bool()
        enable_msg.data = True
        self.scan_enable_pub.publish(enable_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OscillationController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
