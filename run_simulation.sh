#!/bin/bash
# One-click simulation runner for maze drone
# Requires: ROS2 Humble, PX4-Autopilot, Gazebo Harmonic, MicroXRCEAgent

set -e

WORKSPACE=~/autonomous_maze_drone
PX4_DIR=~/PX4-Autopilot

echo "=== Autonomous Maze Drone Simulation ==="

# Check deps
if [ ! -f "/opt/ros/humble/setup.bash" ]; then echo "ROS2 Humble not found"; exit 1; fi
if [ ! -d "$PX4_DIR" ]; then echo "PX4 not found at $PX4_DIR"; exit 1; fi

# Source workspace
source /opt/ros/humble/setup.bash
source $WORKSPACE/install/setup.bash || (echo "Build workspace first: colcon build"; exit 1)

# Copy models
mkdir -p ~/.gz/models/
cp -r $WORKSPACE/src/drone_description/models/x500_maze_explorer ~/.gz/models/ 2>/dev/null || true
mkdir -p $PX4_DIR/Tools/simulation/gz/worlds/
cp $WORKSPACE/src/drone_description/worlds/maze.sdf $PX4_DIR/Tools/simulation/gz/worlds/ 2>/dev/null || true
cp $WORKSPACE/src/px4_config/4015_gz_x500_maze_explorer $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/4015_gz_x500_maze_explorer 2>/dev/null || true

echo "Starting simulation in tmux..."

# Use tmux for 5 terminals
SESSION=maze_drone_sim
tmux kill-session -t $SESSION 2>/dev/null || true
tmux new-session -d -s $SESSION

# Pane 0: MicroXRCE Agent
tmux send-keys -t $SESSION:0 "MicroXRCEAgent udp4 -p 8888" C-m

# Pane 1: PX4 SITL
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "cd $PX4_DIR && PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=maze ./build/px4_sitl_default/bin/px4" C-m

# Pane 2: ROS2 bridges
tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.2 "sleep 5 && source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash && ros2 launch drone_bringup gz_bridge.launch.py" C-m

# Pane 3: Autonomy stack
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.3 "sleep 10 && source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash && ros2 launch drone_bringup autonomy.launch.py" C-m

# Pane 4: Offboard controller
tmux new-window -t $SESSION:1
tmux send-keys -t $SESSION:1 "sleep 15 && source /opt/ros/humble/setup.bash && source $WORKSPACE/install/setup.bash && ros2 run offboard_control offboard_controller --ros-args -p use_vio:=true" C-m

echo "Tmux session $SESSION started"
echo "Attach with: tmux attach -t $SESSION"
echo "Check topics: ros2 topic list"
echo "View map: ros2 topic echo /map --once"
echo "View persons: ros2 topic echo /person_detections"

# Also provide manual commands
cat << EOF
Manual 5-terminal method:

Terminal 1: MicroXRCEAgent udp4 -p 8888
Terminal 2: cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=maze ./build/px4_sitl_default/bin/px4
Terminal 3: source ~/autonomous_maze_drone/install/setup.bash && ros2 launch drone_bringup gz_bridge.launch.py
Terminal 4: source ~/autonomous_maze_drone/install/setup.bash && ros2 launch drone_bringup autonomy.launch.py
Terminal 5: source ~/autonomous_maze_drone/install/setup.bash && ros2 run offboard_control offboard_controller

EOF
