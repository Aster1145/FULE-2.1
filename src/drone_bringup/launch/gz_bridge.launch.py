#!/usr/bin/env python3
"""
Gazebo Harmonic to ROS2 bridge for maze drone
Bridges: /scan (RPLidar), /camera/*, /imu, /air_pressure, /magnetometer
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bridge_config = DeclareLaunchArgument('bridge_config', default_value='', description='YAML bridge config path')

    # ROS_GZ Bridge for sensors
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=[
            # RPLidar
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # IMU
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # Cameras
            '/camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/rgb/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/imu', '/imu/data'),
        ]
    )

    # Image bridge for RGB (ros_gz_image)
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=['/camera/rgb/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        gz_bridge,
        image_bridge
    ])
