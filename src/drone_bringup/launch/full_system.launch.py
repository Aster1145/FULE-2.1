#!/usr/bin/env python3
"""
full_system.launch.py - Complete system for simulation or real drone

Modes:
- simulation: Gazebo + PX4 SITL + VIO sim + exploration
- real: RealSense + RPLidar + Pixhawk + Jetson

Usage:
ros2 launch drone_bringup full_system.launch.py mode:=simulation
ros2 launch drone_bringup full_system.launch.py mode:=real use_rplidar:=true use_realsense:=true
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    use_rplidar = LaunchConfiguration('use_rplidar').perform(context)
    use_realsense = LaunchConfiguration('use_realsense').perform(context)

    print(f"Launching in mode: {mode}, rplidar: {use_rplidar}, realsense: {use_realsense}")

    # Common nodes
    actions = []

    # GZ Bridge (only sim)
    if mode == 'simulation':
        gz_bridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('drone_bringup'), 'launch', 'gz_bridge.launch.py')
            )
        )
        actions.append(gz_bridge_launch)

    # Autonomy stack
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('drone_bringup'), 'launch', 'autonomy.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true' if mode == 'simulation' else 'false',
            'use_rplidar': use_rplidar,
            'use_realsense': use_realsense,
        }.items()
    )
    actions.append(autonomy_launch)

    # Offboard controller
    offboard_controller = Node(
        package='offboard_control',
        executable='offboard_controller.py',
        name='offboard_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True if mode == 'simulation' else False,
            'use_vio': True,
            'takeoff_height': -1.5,
        }]
    )
    actions.append(offboard_controller)

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='simulation', description='simulation or real'),
        DeclareLaunchArgument('use_rplidar', default_value='true', description='Use RPLidar'),
        DeclareLaunchArgument('use_realsense', default_value='false', description='Use RealSense for VIO'),
        OpaqueFunction(function=launch_setup)
    ])
