# QUICKSTART - 5 Minutes to Simulation

## Prerequisites
Ubuntu 22.04 + ROS2 Humble + Gazebo Harmonic + PX4 Autopilot (see install_dependencies.sh)

## 1. Clone & Build
```bash
cd ~
git clone <this-repo> autonomous_maze_drone
cd autonomous_maze_drone
source /opt/ros/humble/setup.bash
# External deps
mkdir -p src/external
cd src/external
git clone https://github.com/Slamtec/rplidar_ros.git -b ros2
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ../..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 2. Setup PX4 Model
```bash
cp -r src/drone_description/models/x500_maze_explorer ~/.gz/models/
cp src/drone_description/worlds/maze.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/maze.sdf
cp src/px4_config/4015_gz_x500_maze_explorer ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
```

## 3. Run (5 terminals or use tmux script)

**Terminal 1:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2:**
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=maze ./build/px4_sitl_default/bin/px4
```

**Terminal 3:**
```bash
source ~/autonomous_maze_drone/install/setup.bash
ros2 launch drone_bringup gz_bridge.launch.py
```

**Terminal 4:**
```bash
source ~/autonomous_maze_drone/install/setup.bash
ros2 launch drone_bringup autonomy.launch.py
```

**Terminal 5:**
```bash
source ~/autonomous_maze_drone/install/setup.bash
ros2 run offboard_control offboard_controller --ros-args -p use_vio:=true
```

**After 10 sec, in new terminal:**
```bash
source ~/autonomous_maze_drone/install/setup.bash
ros2 run fuel_ros2_exploration exploration_manager
```

## 4. Visualize
```bash
rviz2
# Add topics: /map, /voxel_map, /frontiers, /planner/viz, /person_markers
```

## 5. Check Results
```bash
cat /tmp/person_detections.json
ls /tmp/maze_map*
# Map saved as /tmp/maze_map_with_persons.png
```

## One-liner tmux
```bash
./run_simulation.sh
tmux attach -t maze_drone_sim
```

## Real Drone
```bash
# On Jetson Orin Nano
./edge_deploy/jetson_setup.sh
colcon build
ros2 launch drone_bringup full_system.launch.py mode:=real use_rplidar:=true use_realsense:=true
```
