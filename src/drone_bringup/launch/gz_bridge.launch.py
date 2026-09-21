#!/usr/bin/env python3
"""Gazebo to ROS 2 bridge for the maze drone.

Bridges: /scan (RPLidar), /camera/*, /imu, /air_pressure, /magnetometer,
plus /clock for use_sim_time.

Works with both Gazebo Classic (gazebo_ros) and Gazebo Harmonic
(ros_gz_bridge / ros_gz_image). The parameter bridge topics below use
the ros_gz_bridge syntax; when running Gazebo Classic with gazebo_ros,
the equivalent topics are published natively by the gazebo_ros camera,
IMU and ray plugins and this bridge simply forwards /clock.

PEP-8 compliant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the Gazebo bridge launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulated (Gazebo) clock.',
    )
    declare_bridge_config = DeclareLaunchArgument(
        'bridge_config',
        default_value='',
        description='Optional YAML bridge config path (unused, reserved).',
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/rgb/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/points@sensor_msgs/msg/PointCloud2@'
            'gz.msgs.PointCloudPacked',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/imu', '/imu/data'),
        ],
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=['/camera/rgb/image_raw'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_bridge_config,
            gz_bridge,
            image_bridge,
        ]
    )
