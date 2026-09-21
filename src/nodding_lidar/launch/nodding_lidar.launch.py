#!/usr/bin/env python3
"""Nodding lidar launch: oscillation, 2D-to-3D and hardware mechanism.

PEP-8 compliant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the nodding lidar launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulated (Gazebo) clock.',
    )

    oscillation_controller = Node(
        package='nodding_lidar',
        executable='oscillation_controller.py',
        name='oscillation_controller',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'min_angle_deg': -45.0,
                'max_angle_deg': 45.0,
                'frequency_hz': 0.5,
                'mode': 'sinusoidal',
            }
        ],
    )

    scan_to_3d = Node(
        package='nodding_lidar',
        executable='scan_to_3d.py',
        name='scan_to_3d',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'accumulate_scans': 20,
                'min_range': 0.15,
                'max_range': 12.0,
            }
        ],
    )

    nodding_mechanism = Node(
        package='nodding_lidar',
        executable='nodding_mechanism.py',
        name='nodding_mechanism',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'hardware': 'sim',
            }
        ],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            oscillation_controller,
            scan_to_3d,
            nodding_mechanism,
        ]
    )
