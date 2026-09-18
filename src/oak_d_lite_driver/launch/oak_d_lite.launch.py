#!/usr/bin/env python3
"""
OAK-D Lite launch for NIDAR AirMouse
Uses depthai_ros_driver with VIO + RGB + stereo
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('oak_d_lite_driver'),
        'config',
        'oak_d_lite_vio.yaml'
    )

    # DepthAI ROS driver node (monolithic)
    oak_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak',
        parameters=[config_file],
        output='screen',
        # Remap to standard topics expected by VIO and YOLO
        remappings=[
            ('/oak/rgb/image_raw', '/camera/rgb/image_raw'),
            ('/oak/rgb/camera_info', '/camera/rgb/camera_info'),
            ('/oak/stereo/image_raw', '/camera/stereo/image_raw'),
            ('/oak/left/image_raw', '/camera/left/image_raw'),
            ('/oak/right/image_raw', '/camera/right/image_raw'),
            ('/oak/left/camera_info', '/camera/left/camera_info'),
            ('/oak/right/camera_info', '/camera/right/camera_info'),
            ('/oak/stereo/camera_info', '/camera/stereo/camera_info'),
            ('/oak/imu/data', '/imu/data'),
            ('/oak/imu/mag', '/imu/mag'),
        ]
    )

    # SpectacularAI VIO node (if using OAK-D Lite with IMU)
    # Alternative: VINS-Fusion ROS2 subscribing to left/right + IMU
    # Here we provide both options, comment one

    # Option 1: SpectacularAI VIO (requires pip install spectacularAI depthai)
    # This would be a custom node, we provide placeholder
    # vio_node = Node(
    #     package='oak_d_lite_driver',
    #     executable='spectacular_vio_node.py',
    #     name='oak_vio',
    #     output='screen'
    # )

    # Option 2: VINS-Fusion for OAK-D Lite stereo + IMU
    # Requires vins_fusion_ros2 built
    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('oak_d_lite_driver'), 'config', 'oak_d_lite_vio_config.yaml')
        }],
        remappings=[
            ('/camera/left/image_raw', '/camera/left/image_raw'),
            ('/camera/right/image_raw', '/camera/right/image_raw'),
            ('/imu/data', '/imu/data'),
        ]
    )

    return LaunchDescription([
        oak_node,
        # vins_node  # Uncomment if using VINS
    ])
