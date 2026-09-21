#!/usr/bin/env python3
"""Autonomy launch: VIO, mapping, exploration, detection.

Starts mapping, frontier detection, planning, exploration management,
VIO bridging, YOLO detection, 2D map saving and SLAM toolbox.

Launch arguments:
  use_sim_time: use Gazebo clock (default true).
  is_sim: simulation mode, disables hardware-only branches (default true).
  use_rplidar / use_realsense: real-drone sensor selection (default false).

PEP-8 compliant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the autonomy launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_sim = LaunchConfiguration('is_sim')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulated (Gazebo) clock.',
    )
    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description='Simulation mode (disables hardware-only nodes).',
    )
    declare_use_rplidar = DeclareLaunchArgument(
        'use_rplidar',
        default_value='false',
        description='Use physical RPLidar (true on real drone).',
    )
    declare_use_realsense = DeclareLaunchArgument(
        'use_realsense',
        default_value='false',
        description='Use physical RealSense (true on real drone).',
    )

    mapping_node = Node(
        package='fuel_ros2_exploration',
        executable='mapping_node.py',
        name='mapping_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'map_size': 20.0,
                'resolution': 0.1,
                'origin_x': -10.0,
                'origin_y': -10.0,
            }
        ],
    )

    frontier_detector = Node(
        package='fuel_ros2_exploration',
        executable='frontier_detector.py',
        name='frontier_detector',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'min_frontier_size': 5,
                'sensor_range': 4.0,
            }
        ],
    )

    planner_node = Node(
        package='fuel_ros2_exploration',
        executable='planner_node.py',
        name='planner_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'planning_height': 1.5,
                'max_vel': 1.0,
            }
        ],
    )

    exploration_manager = Node(
        package='fuel_ros2_exploration',
        executable='exploration_manager.py',
        name='exploration_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'exploration_timeout': 600.0,
                'home_x': 0.0,
                'home_y': 0.0,
            }
        ],
    )

    vio_bridge = Node(
        package='offboard_control',
        executable='vio_bridge.py',
        name='vio_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    yolo_detector = Node(
        package='person_detection',
        executable='yolo_detector.py',
        name='yolo_detector',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'model': 'yolov8n.pt',
                'conf_thres': 0.5,
                'image_topic': '/camera/rgb/image_raw',
                'device': 'cpu',
                'use_tensorrt': False,
            }
        ],
    )

    map_saver = Node(
        package='maze_mapping',
        executable='map_saver_with_persons.py',
        name='map_saver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'resolution': 0.05,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_is_sim,
            declare_use_rplidar,
            declare_use_realsense,
            mapping_node,
            frontier_detector,
            planner_node,
            exploration_manager,
            vio_bridge,
            yolo_detector,
            map_saver,
            slam_toolbox,
        ]
    )
