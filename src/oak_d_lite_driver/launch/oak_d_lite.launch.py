#!/usr/bin/env python3
"""OAK-D Lite launch for NIDAR AirMouse.

Uses depthai_ros_driver with VIO + RGB + stereo.

Hardware decoupling:
  - ``is_sim:=true`` disables the physical depthai_ros_driver entirely
    and relies on the simulated Gazebo camera plugin topics
    (``/camera/rgb/image_raw`` and friends). This prevents the
    exit-code -6 crash seen in VMs without the camera plugged in.
  - ``is_sim:=false`` starts the physical driver on the real drone.

Usage:
  # Simulation (driver disabled, Gazebo cameras used):
  ros2 launch oak_d_lite_driver oak_d_lite.launch.py is_sim:=true

  # Real hardware:
  ros2 launch oak_d_lite_driver oak_d_lite.launch.py is_sim:=false

PEP-8 compliant.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the OAK-D Lite launch description."""
    is_sim = LaunchConfiguration('is_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='false',
        description=(
            'Simulation mode. When true, the physical depthai_ros_driver '
            'is disabled and Gazebo camera plugins are used instead.'
        ),
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated (Gazebo) clock.',
    )

    config_file = os.path.join(
        get_package_share_directory('oak_d_lite_driver'),
        'config',
        'oak_d_lite_vio.yaml',
    )

    # Physical DepthAI ROS driver. Disabled in simulation.
    oak_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(is_sim),
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
        ],
    )

    # Optional VINS-Fusion estimator for OAK-D Lite stereo + IMU.
    # Requires vins_fusion_ros2 built. Disabled by default; also gated
    # behind UnlessCondition(is_sim) so sim never touches hardware.
    vins_node = Node(  # noqa: F841 - enable on hardware below
        package='vins',
        executable='vins_node',
        name='vins_estimator',
        output='screen',
        parameters=[
            {
                'config_file': os.path.join(
                    get_package_share_directory('oak_d_lite_driver'),
                    'config',
                    'oak_d_lite_vio_config.yaml',
                ),
                'use_sim_time': use_sim_time,
            }
        ],
        remappings=[
            ('/camera/left/image_raw', '/camera/left/image_raw'),
            ('/camera/right/image_raw', '/camera/right/image_raw'),
            ('/imu/data', '/imu/data'),
        ],
        # Uncomment to enable VINS on hardware. Keep disabled in sim.
        # condition=UnlessCondition(is_sim),
    )

    return LaunchDescription(
        [
            declare_is_sim,
            declare_use_sim_time,
            oak_node,
            # vins_node,  # Uncomment if using VINS on hardware.
        ]
    )
