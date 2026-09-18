#!/usr/bin/env python3
"""
autonomy.launch.py - Launch VIO, mapping, exploration, detection
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Config
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rplidar = LaunchConfiguration('use_rplidar', default='false')  # true on real drone
    use_realsense = LaunchConfiguration('use_realsense', default='false')

    # Mapping node (FUEL-inspired)
    mapping_node = Node(
        package='fuel_ros2_exploration',
        executable='mapping_node.py',
        name='mapping_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_size': 20.0,
            'resolution': 0.1,
            'origin_x': -10.0,
            'origin_y': -10.0,
        }]
    )

    # Frontier detector (FIS)
    frontier_detector = Node(
        package='fuel_ros2_exploration',
        executable='frontier_detector.py',
        name='frontier_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_frontier_size': 5,
            'sensor_range': 4.0,
        }]
    )

    # Planner (FUEL hierarchical)
    planner_node = Node(
        package='fuel_ros2_exploration',
        executable='planner_node.py',
        name='planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planning_height': 1.5,
            'max_vel': 1.0,
        }]
    )

    # Exploration manager FSM
    exploration_manager = Node(
        package='fuel_ros2_exploration',
        executable='exploration_manager.py',
        name='exploration_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'exploration_timeout': 600.0,
            'home_x': 0.0,
            'home_y': 0.0,
        }]
    )

    # VIO Bridge
    vio_bridge = Node(
        package='offboard_control',
        executable='vio_bridge.py',
        name='vio_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # YOLO person detection
    yolo_detector = Node(
        package='person_detection',
        executable='yolo_detector.py',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'model': 'yolov8n.pt',
            'conf_thres': 0.5,
            'image_topic': '/camera/rgb/image_raw',
            'device': 'cpu',  # change to 'cuda:0' on Jetson
            'use_tensorrt': False,
        }]
    )

    # Map saver with persons
    map_saver = Node(
        package='maze_mapping',
        executable='map_saver_with_persons.py',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # slam_toolbox for 2D mapping from RPLidar (if available)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'resolution': 0.05,
        }]
    )

    return LaunchDescription([
        mapping_node,
        frontier_detector,
        planner_node,
        exploration_manager,
        vio_bridge,
        yolo_detector,
        map_saver,
        slam_toolbox
    ])
