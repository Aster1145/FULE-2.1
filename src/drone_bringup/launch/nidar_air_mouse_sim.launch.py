#!/usr/bin/env python3
"""NIDAR AirMouse simulation launch.

Arena: 15x15m maze, 6 survivors, simulated OAK-D Lite cameras,
nodding lidar, autonomy stack, mission manager and offboard control.

This file is the single entry point for simulation on the Ubuntu 22.04
ARM64 VM (ROS 2 Humble, Gazebo 11, X11 + software rendering).

What it starts, in order:
  1. Gazebo Classic physics server (gzserver) with nidar_air_mouse.world.
  2. ROS-Gazebo bridges (scan, cameras, IMU, clock).
  3. Nodding lidar (oscillation + scan_to_3d).
  4. Autonomy stack (mapping, frontier, planner, YOLO, map saver).
  5. AirMouse mission nodes (mission manager, survivor detector,
     failsafe, map generator) with physical hardware disabled.
  6. Offboard controller (PX4, VIO mode).

Hardware decoupling:
  - ``is_sim:=true`` (default) disables the physical OAK-D Lite driver
    (depthai_ros_driver) and relies on the simulated Gazebo camera
    plugin topics (``/camera/rgb/image_raw`` and friends).
  - Set ``is_sim:=false`` only on the real drone with the camera plugged
    in.

VM display notes:
  - The launch sets ``QT_QPA_PLATFORM=xcb`` and
    ``LIBGL_ALWAYS_SOFTWARE=1`` for UTM / Apple Silicon VMs.
  - ``start_gzclient`` defaults to false (headless). Visualize with RViz.

Usage:
  ros2 launch drone_bringup nidar_air_mouse_sim.launch.py
  ros2 launch drone_bringup nidar_air_mouse_sim.launch.py gui:=true
  ros2 launch drone_bringup nidar_air_mouse_sim.launch.py \\
      start_gzserver:=false  # if PX4 SITL already started Gazebo

PEP-8 compliant (4 spaces, no tabs).
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WORLD_FILENAME_CLASSIC = 'nidar_air_mouse.world'
WORLD_FILENAME_GZ = 'nidar_air_mouse.sdf'


def _resolve_default_world() -> str:
    """Return the default world file path for gzserver.

    Prefers the Gazebo Classic ``.world`` file, falls back to the
    Harmonic ``.sdf`` file so both simulators keep working.
    """
    try:
        worlds_dir = os.path.join(
            get_package_share_directory('drone_description'), 'worlds'
        )
    except PackageNotFoundError:
        # Colcon source fallback (launch file run from repo without install).
        here = os.path.dirname(os.path.abspath(__file__))
        worlds_dir = os.path.normpath(
            os.path.join(
                here, '..', '..', 'drone_description', 'worlds',
            )
        )
    classic = os.path.join(worlds_dir, WORLD_FILENAME_CLASSIC)
    if os.path.exists(classic):
        return classic
    return os.path.join(worlds_dir, WORLD_FILENAME_GZ)


def _package_share_or_none(package_name: str):
    """Return share dir or None if the package is not installed."""
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        return None


def _gzserver_include(world_config: LaunchConfiguration):
    """Build the gzserver include (Gazebo Classic preferred).

    Returns an IncludeLaunchDescription for ``gazebo_ros/gzserver`` when
    available, otherwise falls back to ``ros_gz_sim/gz_sim`` for
    Harmonic. Execution is gated by the ``start_gzserver`` argument.
    """
    start_condition = IfCondition(LaunchConfiguration('start_gzserver'))
    gazebo_ros_share = _package_share_or_none('gazebo_ros')
    if gazebo_ros_share is not None:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world_config,
                'verbose': LaunchConfiguration('verbose'),
            }.items(),
            condition=start_condition,
        )

    ros_gz_sim_share = _package_share_or_none('ros_gz_sim')
    if ros_gz_sim_share is not None:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -v 3 ', world_config],
            }.items(),
            condition=start_condition,
        )

    return LogInfo(
        msg=(
            '[nidar_air_mouse_sim] Neither gazebo_ros nor ros_gz_sim found; '
            'skipping physics server. Start Gazebo manually.'
        ),
        condition=start_condition,
    )


def _gzclient_include():
    """Build the optional gzclient include for Gazebo Classic."""
    gazebo_ros_share = _package_share_or_none('gazebo_ros')
    if gazebo_ros_share is None:
        return LogInfo(
            msg=(
                '[nidar_air_mouse_sim] gazebo_ros not found; '
                'gzclient unavailable.'
            )
        )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_gzclient')),
    )


def generate_launch_description() -> LaunchDescription:
    """Generate the NIDAR AirMouse simulation launch description."""
    default_world = _resolve_default_world()
    drone_bringup_share = _package_share_or_none('drone_bringup')
    nodding_share = _package_share_or_none('nodding_lidar')
    air_mouse_share = _package_share_or_none('air_mouse_mission')

    # Launch arguments.
    use_sim_time = LaunchConfiguration('use_sim_time')
    is_sim = LaunchConfiguration('is_sim')
    world = LaunchConfiguration('world')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulated (Gazebo) clock for all ROS nodes.',
    )
    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description=(
            'Simulation mode. When true, physical hardware drivers '
            '(depthai_ros_driver) are disabled and Gazebo camera '
            'plugins are used instead.'
        ),
    )
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to nidar_air_mouse.world for gzserver.',
    )
    declare_start_gzserver = DeclareLaunchArgument(
        'start_gzserver',
        default_value='true',
        description='Start the Gazebo physics server (gzserver).',
    )
    declare_start_gzclient = DeclareLaunchArgument(
        'start_gzclient',
        default_value='false',
        description=(
            'Start gzclient GUI (false = headless, recommended in the VM).'
        ),
    )
    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Deprecated alias for start_gzclient.',
    )
    declare_verbose = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Run gzserver with verbose output.',
    )
    declare_takeoff_height = DeclareLaunchArgument(
        'takeoff_height',
        default_value='-1.5',
        description='Offboard takeoff setpoint (NED, metres).',
    )

    # VM-friendly rendering: X11 + software GL (UTM / Apple Silicon).
    set_qt_platform = SetEnvironmentVariable(
        name='QT_QPA_PLATFORM', value='xcb',
    )
    set_software_gl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE', value='1',
    )

    # 1. Physics server (gzserver) with nidar_air_mouse.world.
    gzserver = _gzserver_include(world)
    gzclient = _gzclient_include()

    # 2. ROS-Gazebo bridges for sim sensors.
    gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                drone_bringup_share, 'launch', 'gz_bridge.launch.py',
            )
            if drone_bringup_share
            else 'gz_bridge.launch.py'
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 3. Nodding lidar sim (oscillation + scan_to_3d).
    nodding = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nodding_share, 'launch', 'nodding_lidar.launch.py',
            )
            if nodding_share
            else 'nodding_lidar.launch.py'
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 4. Autonomy (mapping, frontier, planner) on sim time.
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                drone_bringup_share, 'launch', 'autonomy.launch.py',
            )
            if drone_bringup_share
            else 'autonomy.launch.py'
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'is_sim': is_sim,
        }.items(),
    )

    # 5. AirMouse mission (hardware disabled when is_sim=true).
    air_mouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                air_mouse_share, 'launch', 'air_mouse.launch.py',
            )
            if air_mouse_share
            else 'air_mouse.launch.py'
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'is_sim': is_sim,
        }.items(),
    )

    # 6. Offboard controller (PX4 SITL + VIO).
    offboard = Node(
        package='offboard_control',
        executable='offboard_controller.py',
        name='offboard_controller',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'use_vio': True,
                'takeoff_height': LaunchConfiguration('takeoff_height'),
            }
        ],
    )

    sim_info = LogInfo(
        msg=[
            '[nidar_air_mouse_sim] is_sim:=',
            is_sim,
            ' use_sim_time:=',
            use_sim_time,
            ' world:=',
            world,
        ]
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_is_sim,
            declare_world,
            declare_start_gzserver,
            declare_start_gzclient,
            declare_gui,
            declare_verbose,
            declare_takeoff_height,
            set_qt_platform,
            set_software_gl,
            sim_info,
            gzserver,
            gzclient,
            gz_bridge,
            nodding,
            autonomy,
            air_mouse,
            offboard,
        ]
    )
