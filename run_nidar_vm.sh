#!/bin/bash
# One-click runner for NIDAR AirMouse C++ stack in Ubuntu 22.04 VM on MacBook
# Usage: ./run_nidar_vm.sh

set -e

WORKSPACE=~/autonomous_maze_drone
PX4_DIR=~/PX4-Autopilot

echo "=== NIDAR AirMouse C++ VM Runner ==="

# Check
if [ ! -f "/opt/ros/humble/setup.bash" ]; then echo "ROS2 Humble not found. Run install_dependencies.sh"; exit 1; fi
if [ ! -d "$PX4_DIR" ]; then echo "PX4 not found at $PX4_DIR"; exit 1; fi
if [ ! -f "$WORKSPACE/install/setup.bash" ]; then echo "Build workspace first: ./build_cpp.sh"; exit 1; fi

source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash

# Copy models/worlds
mkdir -p ~/.gz/models/
cp -r $WORKSPACE/src/drone_description/models/x500_maze_explorer ~/.gz/models/ 2>/dev/null || true
mkdir -p $PX4_DIR/Tools/simulation/gz/worlds/
cp $WORKSPACE/src/drone_description/worlds/nidar_air_mouse.sdf $PX4_DIR/Tools/simulation/gz/worlds/ 2>/dev/null || true
cp $WORKSPACE/src/drone_description/worlds/maze.sdf $PX4_DIR/Tools/simulation/gz/worlds/ 2>/dev/null || true

# Check ONNX
if [ ! -f "$WORKSPACE/src/air_mouse_cpp/config/yolov8n.onnx" ]; then
  echo "WARNING: yolov8n.onnx not found. Exporting..."
  cd /tmp
  yolo export model=yolov8n.pt format=onnx opset=12 2>/dev/null || echo "Install ultralytics: pip install ultralytics"
  mkdir -p $WORKSPACE/src/air_mouse_cpp/config/
  cp yolov8n.onnx $WORKSPACE/src/air_mouse_cpp/config/ 2>/dev/null || true
  cd $WORKSPACE
fi

echo "Starting NIDAR C++ simulation in tmux..."

SESSION=nidar_cpp_vm
tmux kill-session -t $SESSION 2>/dev/null || true
tmux new-session -d -s $SESSION

# Pane 0: DDS Agent
tmux send-keys -t $SESSION:0 "MicroXRCEAgent udp4 -p 8888" C-m

# Pane 1: PX4 SITL NIDAR World Headless (VM friendly)
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "cd $PX4_DIR && export LIBGL_ALWAYS_SOFTWARE=1 && HEADLESS=1 PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=nidar_air_mouse ./build/px4_sitl_default/bin/px4" C-m

# Pane 2: ROS2 Bridges + Nodding Lidar C++
tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.2 "sleep 5 && source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash && ros2 launch drone_bringup gz_bridge.launch.py & sleep 2 && ros2 run nodding_lidar_cpp oscillation_controller --ros-args -p min_angle_deg:=-45 -p max_angle_deg:=45 -p frequency_hz:=0.5 & ros2 run nodding_lidar_cpp scan_to_3d --ros-args -p accumulate_scans:=20" C-m

# Pane 3: C++ Autonomy Stack
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.3 "sleep 10 && source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash && ros2 run fuel_exploration_cpp mapping_node --ros-args -p map_size:=15.0 -p resolution:=0.1 -p origin_x:=-7.5 -p origin_y:=-7.5 & ros2 run fuel_exploration_cpp frontier_detector & ros2 run fuel_exploration_cpp planner_node & ros2 run air_mouse_cpp survivor_detector --ros-args -p model:=$WORKSPACE/src/air_mouse_cpp/config/yolov8n.onnx -p conf_thres:=0.4 & ros2 run air_mouse_cpp map_generator & ros2 run air_mouse_cpp failsafe --ros-args -p max_height:=2.44 -p arena_size:=15.0 &" C-m

# Window 1: Offboard + Mission
tmux new-window -t $SESSION:1
tmux send-keys -t $SESSION:1 "sleep 15 && source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash && ros2 run offboard_control_cpp offboard_controller --ros-args -p takeoff_height:=-1.5 & ros2 run offboard_control_cpp vio_bridge & sleep 3 && ros2 run air_mouse_cpp mission_manager --ros-args -p arena_size:=15.0 -p max_mission_time:=1800.0 -p max_survivors:=6" C-m

echo "Tmux session $SESSION started"
echo "Attach: tmux attach -t $SESSION"
echo "Check: ros2 topic echo /mission/state"
echo "Map: /tmp/nidar_2d_map_with_survivors.png"
echo ""
echo "Manual 5-terminal method in RUN_IN_UBUNTU_VM_GUIDE.md"
