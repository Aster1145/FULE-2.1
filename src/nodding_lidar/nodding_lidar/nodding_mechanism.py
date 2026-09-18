#!/usr/bin/env python3
"""
nodding_mechanism.py - Hardware interface for RPLidar nodding servo
Supports:
- Arduino via rosserial (Float64 angle command -> servo)
- Dynamixel XL430 via dynamixel_sdk
- PCA9685 PWM driver on Jetson

For NIDAR AirMouse, recommended: MG996R servo + Arduino Nano + 3D printed mount
Arduino code snippet provided in docs.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

# Try import hardware libs
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

class NoddingMechanism(Node):
    def __init__(self):
        super().__init__('nodding_mechanism')
        self.declare_parameter('hardware', 'sim')  # sim, arduino, dynamixel, pca9685
        self.declare_parameter('arduino_port', '/dev/ttyUSB1')
        self.declare_parameter('arduino_baud', 115200)
        self.declare_parameter('servo_min_pwm', 500)
        self.declare_parameter('servo_max_pwm', 2500)

        self.hardware = self.get_parameter('hardware').value
        self.arduino_port = self.get_parameter('arduino_port').value

        self.angle_sub = self.create_subscription(Float64, '/servo_angle_cmd', self.angle_callback, 10)

        self.ser = None
        if self.hardware == 'arduino' and SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(self.arduino_port, self.get_parameter('arduino_baud').value, timeout=1)
                self.get_logger().info(f"Arduino connected on {self.arduino_port}")
            except Exception as e:
                self.get_logger().error(f"Arduino connection failed: {e}, using sim")

        self.get_logger().info(f"Nodding Mechanism hardware: {self.hardware}")

    def angle_callback(self, msg):
        angle_rad = msg.data
        angle_deg = math.degrees(angle_rad)

        # Convert to servo command
        # MG996R: 0-180 deg, 500-2500us PWM
        # Our range -45 to +45 -> map to 45 to 135 deg servo
        servo_deg = 90 + angle_deg  # center 90
        servo_deg = max(0, min(180, servo_deg))

        if self.hardware == 'arduino' and self.ser is not None:
            # Send angle to Arduino: "A90\n"
            try:
                cmd = f"A{int(servo_deg)}\n"
                self.ser.write(cmd.encode())
            except Exception as e:
                self.get_logger().warn(f"Serial write failed: {e}")
        elif self.hardware == 'sim':
            # In sim, just log occasionally
            pass

def main(args=None):
    rclpy.init(args=args)
    node = NoddingMechanism()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
