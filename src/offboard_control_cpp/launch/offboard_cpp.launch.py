from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='offboard_control_cpp', executable='offboard_controller', name='offboard_controller', output='screen',
             parameters=[{'takeoff_height': -1.5}]),
        Node(package='offboard_control_cpp', executable='vio_bridge', name='vio_bridge', output='screen'),
    ])
