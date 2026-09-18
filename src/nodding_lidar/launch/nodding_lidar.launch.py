from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    oscillation_controller = Node(
        package='nodding_lidar',
        executable='oscillation_controller.py',
        name='oscillation_controller',
        output='screen',
        parameters=[{
            'min_angle_deg': -45.0,
            'max_angle_deg': 45.0,
            'frequency_hz': 0.5,
            'mode': 'sinusoidal',
        }]
    )

    scan_to_3d = Node(
        package='nodding_lidar',
        executable='scan_to_3d.py',
        name='scan_to_3d',
        output='screen',
        parameters=[{
            'accumulate_scans': 20,
            'min_range': 0.15,
            'max_range': 12.0,
        }]
    )

    nodding_mechanism = Node(
        package='nodding_lidar',
        executable='nodding_mechanism.py',
        name='nodding_mechanism',
        output='screen',
        parameters=[{
            'hardware': 'sim',  # change to arduino on real drone
        }]
    )

    return LaunchDescription([
        oscillation_controller,
        scan_to_3d,
        nodding_mechanism
    ])
