#!/bin/bash
# Build C++ version for NIDAR competition - fast
set -e
echo "=== Building C++ Fast Stack for NIDAR AirMouse ==="

# Check ROS2
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo "ROS2 Humble not found. Run install_dependencies.sh first"
  exit 1
fi

source /opt/ros/humble/setup.bash

# Install deps
sudo apt update
sudo apt install -y libopencv-dev libopencv-contrib-dev libeigen3-dev libyaml-cpp-dev libceres-dev libpcl-dev
sudo apt install -y ros-humble-px4-msgs ros-humble-depthai-ros

# For map generator YAML + OpenCV
sudo apt install -y libyaml-cpp-dev libopencv-dev

# Build only C++ packages (fast)
echo "Building C++ packages with -O3 -march=native..."
colcon build --packages-select fuel_exploration_cpp offboard_control_cpp nodding_lidar_cpp air_mouse_cpp oak_d_lite_cpp \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native -std=c++17" --parallel-workers $(nproc)

# Build all if needed
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

echo "=== C++ Build Complete ==="
echo "Nodes:"
echo "  fuel_exploration_cpp: mapping_node, frontier_detector, planner_node, exploration_manager"
echo "  offboard_control_cpp: offboard_controller, vio_bridge"
echo "  nodding_lidar_cpp: oscillation_controller, scan_to_3d"
echo "  air_mouse_cpp: mission_manager, survivor_detector, failsafe, map_generator"
echo ""
echo "Run:"
echo "  ros2 launch air_mouse_cpp air_mouse_cpp.launch.py"
echo "  ros2 launch fuel_exploration_cpp fuel_cpp.launch.py"
echo ""
echo "For YOLO ONNX:"
echo "  yolo export model=yolov8n.pt format=onnx opset=12"
echo "  cp yolov8n.onnx src/air_mouse_cpp/config/"
