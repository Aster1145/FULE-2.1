#!/bin/bash
# Test build without external deps (for CI)
set -e
source /opt/ros/humble/setup.bash
cd /home/user/autonomous_maze_drone
rosdep install --from-paths src --ignore-src -r -y --skip-keys="rplidar_ros vins_fusion_ros2 px4_msgs px4_ros_com" || true
colcon build --symlink-install --packages-select drone_description drone_bringup fuel_ros2_exploration offboard_control person_detection maze_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "Build test passed"
