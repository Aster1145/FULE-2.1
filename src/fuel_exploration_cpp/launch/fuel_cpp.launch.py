from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='fuel_exploration_cpp', executable='mapping_node', name='mapping_node', output='screen',
             parameters=[{'map_size': 15.0, 'resolution': 0.1, 'origin_x': -7.5, 'origin_y': -7.5, 'voxel_height': 3.0}]),
        Node(package='fuel_exploration_cpp', executable='frontier_detector', name='frontier_detector', output='screen',
             parameters=[{'min_frontier_size': 8, 'sensor_range': 4.0, 'info_gain_threshold': 10}]),
        Node(package='fuel_exploration_cpp', executable='planner_node', name='planner_node', output='screen',
             parameters=[{'planning_height': 1.5, 'safety_margin': 0.3}]),
        Node(package='fuel_exploration_cpp', executable='exploration_manager', name='exploration_manager', output='screen',
             parameters=[{'exploration_timeout': 600.0, 'home_x': 0.0, 'home_y': 0.0}]),
    ])
