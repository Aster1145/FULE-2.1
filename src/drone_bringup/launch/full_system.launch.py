#!/usr/bin/env python3
"""Full system launch for simulation or the real drone.

Modes:
  - simulation: Gazebo + PX4 SITL + VIO sim + exploration.
  - real: RealSense + RPLidar + Pixhawk + Jetson.

Usage:
  ros2 launch drone_bringup full_system.launch.py mode:=simulation
  ros2 launch drone_bringup full_system.launch.py mode:=real \\
      use_rplidar:=true use_realsense:=true

PEP-8 compliant.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Build launch actions based on the selected mode."""
    mode = LaunchConfiguration('mode').perform(context)
    use_rplidar = LaunchConfiguration('use_rplidar').perform(context)
    use_realsense = LaunchConfiguration('use_realsense').perform(context)
    is_sim = 'true' if mode == 'simulation' else 'false'
    use_sim_time = 'true' if mode == 'simulation' else 'false'

    print(
        f'Launching in mode: {mode}, rplidar: {use_rplidar}, '
        f'realsense: {use_realsense}'
    )

    actions = []

    # GZ bridge (simulation only).
    if mode == 'simulation':
        gz_bridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('drone_bringup'),
                    'launch',
                    'gz_bridge.launch.py',
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )
        actions.append(gz_bridge_launch)

    # Autonomy stack.
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
            'use_rplidar': use_rplidar,
            'use_realsense': use_realsense,
        }.items(),
    )
    actions.append(autonomy_launch)

    # Offboard controller.
    offboard_controller = Node(
        package='offboard_control',
        executable='offboard_controller.py',
        name='offboard_controller',
        output='screen',
        parameters=[
            {
                'use_sim_time': mode == 'simulation',
                'use_vio': True,
                'takeoff_height': -1.5,
            }
        ],
    )
    actions.append(offboard_controller)

    return actions


def generate_launch_description() -> LaunchDescription:
    """Generate the full system launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode',
                default_value='simulation',
                description='simulation or real',
            ),
            DeclareLaunchArgument(
                'use_rplidar',
                default_value='true',
                description='Use RPLidar',
            ),
            DeclareLaunchArgument(
                'use_realsense',
                default_value='false',
                description='Use RealSense for VIO',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
