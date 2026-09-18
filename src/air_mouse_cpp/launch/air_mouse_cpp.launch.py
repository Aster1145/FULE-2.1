from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    fuel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('fuel_exploration_cpp'), 'launch', 'fuel_cpp.launch.py'))
    )
    offboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('offboard_control_cpp'), 'launch', 'offboard_cpp.launch.py'))
    )
    nodding = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nodding_lidar_cpp'), 'launch', 'nodding_cpp.launch.py'))
    )

    return LaunchDescription([
        fuel,
        offboard,
        nodding,
        Node(package='air_mouse_cpp', executable='mission_manager', name='mission_manager', output='screen',
             parameters=[{'arena_size': 15.0, 'max_mission_time': 1800.0, 'max_survivors': 6, 'home_x': 0.0, 'home_y': 0.0}]),
        Node(package='air_mouse_cpp', executable='survivor_detector', name='survivor_detector', output='screen',
             parameters=[{'model': 'yolov8n.onnx', 'conf_thres': 0.4, 'max_survivors': 6}]),
        Node(package='air_mouse_cpp', executable='failsafe', name='failsafe', output='screen',
             parameters=[{'max_height': 2.44, 'arena_size': 15.0}]),
        Node(package='air_mouse_cpp', executable='map_generator', name='map_generator', output='screen'),
    ])
