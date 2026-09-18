#!/usr/bin/env python3
"""
NIDAR AirMouse Simulation Launch - 15x15m maze, 6 survivors, OAK-D Lite sim, nodding lidar
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Use nidar_air_mouse world
    # Bridges for sim
    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('drone_bringup'), 'launch', 'gz_bridge.launch.py')
        )
    )

    # Nodding lidar sim (oscillation + scan_to_3d)
    nodding = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nodding_lidar'), 'launch', 'nodding_lidar.launch.py')
        )
    )

    # Autonomy (mapping, frontier, planner) - sim time
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('drone_bringup'), 'launch', 'autonomy.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # AirMouse mission manager
    air_mouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('air_mouse_mission'), 'launch', 'air_mouse.launch.py')
        )
    )

    # Offboard controller
    offboard = Node(
        package='offboard_control',
        executable='offboard_controller.py',
        name='offboard_controller',
        output='screen',
        parameters=[{'use_sim_time': True, 'use_vio': True, 'takeoff_height': -1.5}]
    )

    return LaunchDescription([
        gz_bridge,
        nodding,
        autonomy,
        air_mouse,
        offboard
    ])
