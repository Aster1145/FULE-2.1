from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='nodding_lidar_cpp', executable='oscillation_controller', name='oscillation_controller', output='screen',
             parameters=[{'min_angle_deg': -45.0, 'max_angle_deg': 45.0, 'frequency_hz': 0.5, 'mode': 'sinusoidal'}]),
        Node(package='nodding_lidar_cpp', executable='scan_to_3d', name='scan_to_3d', output='screen',
             parameters=[{'accumulate_scans': 20, 'min_range': 0.15, 'max_range': 12.0}]),
    ])
