#!/usr/bin/env python3
"""AirMouse mission launch for NIDAR 2026.

Includes:
  - OAK-D Lite driver (hardware only, skipped when is_sim=true)
  - Nodding RPLidar (2D -> 3D)
  - VIO bridge (via autonomy stack)
  - FUEL-inspired exploration (mapping + frontier + planner)
  - Survivor detection (YOLO)
  - Mission manager + failsafe + map generator
  - Offboard control (PX4)

Hardware decoupling:
  - ``is_sim:=true`` (default) disables the physical OAK-D Lite driver
    and uses Gazebo camera plugins instead.
  - ``is_sim:=false`` + ``use_sim_time:=false`` for the real drone.

Usage:
  ros2 launch air_mouse_mission air_mouse.launch.py is_sim:=true
  ros2 launch air_mouse_mission air_mouse.launch.py \\
      is_sim:=false use_sim_time:=false

PEP-8 compliant.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the AirMouse mission launch description."""
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
        description=(
            'Simulation mode. When true, the physical OAK-D Lite driver '
            'is not started.'
        ),
    )

    # OAK-D Lite hardware driver. Skipped in simulation.
    oak_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('oak_d_lite_driver'),
                'launch',
                'oak_d_lite.launch.py',
            )
        ),
        launch_arguments={
            'is_sim': is_sim,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=UnlessCondition(is_sim),
    )

    # Nodding lidar.
    nodding_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nodding_lidar'),
                'launch',
                'nodding_lidar.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Autonomy (mapping + frontier + planner) from drone_bringup.
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('drone_bringup'),
                'launch',
                'autonomy.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'is_sim': is_sim,
        }.items(),
    )

    mission_manager = Node(
        package='air_mouse_mission',
        executable='mission_manager.py',
        name='mission_manager',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'arena_size': 15.0,
                'max_mission_time': 1800.0,
                'max_survivors': 6,
                'home_x': 0.0,
                'home_y': 0.0,
            }
        ],
    )

    survivor_detector = Node(
        package='air_mouse_mission',
        executable='survivor_detector.py',
        name='survivor_detector',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'model': 'yolov8n.pt',
                'conf_thres': 0.4,
                'max_survivors': 6,
            }
        ],
    )

    failsafe = Node(
        package='air_mouse_mission',
        executable='failsafe.py',
        name='failsafe',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'max_height': 2.44,
                'arena_size': 15.0,
                'startup_grace_period': 20.0,
                'require_sim_time': True,
            }
        ],
    )

    map_generator = Node(
        package='air_mouse_mission',
        executable='map_generator.py',
        name='map_generator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    offboard = Node(
        package='offboard_control',
        executable='offboard_controller.py',
        name='offboard_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'use_vio': True},
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_is_sim,
            oak_launch,
            nodding_launch,
            autonomy_launch,
            mission_manager,
            survivor_detector,
            failsafe,
            map_generator,
            offboard,
        ]
    )
