#!/usr/bin/env python3
"""
air_mouse.launch.py - Full NIDAR 2026 AirMouse mission launch
Includes:
- OAK-D Lite driver (stereo + RGB + IMU)
- Nodding RPLidar (2D->3D)
- VIO (VINS-Fusion or SpectacularAI)
- FUEL-inspired exploration (mapping + frontier + planner)
- Survivor detection (YOLO)
- Mission manager + failsafe + map generator
- Offboard control (PX4)
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # OAK-D Lite
    oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('oak_d_lite_driver'), 'launch', 'oak_d_lite.launch.py')
        )
    )

    # Nodding lidar
    nodding_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nodding_lidar'), 'launch', 'nodding_lidar.launch.py')
        )
    )

    # Autonomy (mapping + frontier + planner) - from drone_bringup
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('drone_bringup'), 'launch', 'autonomy.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Mission manager
    mission_manager = Node(
        package='air_mouse_mission',
        executable='mission_manager.py',
        name='mission_manager',
        output='screen',
        parameters=[{
            'arena_size': 15.0,
            'max_mission_time': 1800.0,
            'max_survivors': 6,
            'home_x': 0.0,
            'home_y': 0.0,
        }]
    )

    survivor_detector = Node(
        package='air_mouse_mission',
        executable='survivor_detector.py',
        name='survivor_detector',
        output='screen',
        parameters=[{
            'model': 'yolov8n.pt',
            'conf_thres': 0.4,
            'max_survivors': 6,
        }]
    )

    failsafe = Node(
        package='air_mouse_mission',
        executable='failsafe.py',
        name='failsafe',
        output='screen',
        parameters=[{
            'max_height': 2.44,
            'arena_size': 15.0,
        }]
    )

    map_generator = Node(
        package='air_mouse_mission',
        executable='map_generator.py',
        name='map_generator',
        output='screen'
    )

    # Offboard controller
    offboard = Node(
        package='offboard_control',
        executable='offboard_controller.py',
        name='offboard_controller',
        output='screen',
        parameters=[{'use_vio': True}]
    )

    return LaunchDescription([
        oak_launch,
        nodding_launch,
        autonomy_launch,
        mission_manager,
        survivor_detector,
        failsafe,
        map_generator,
        offboard
    ])
