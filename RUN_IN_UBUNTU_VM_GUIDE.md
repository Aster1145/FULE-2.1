# Run NIDAR AirMouse C++ Stack in Ubuntu 22.04 VM on MacBook - Complete Guide

You cloned https://github.com/Aster1145/FULE-2.1.git - now run it in your Ubuntu 22.04 LTS VM.

---

## 0. VM Specs Check (Do this first)

In your Ubuntu VM terminal:

```bash
# Check Ubuntu version
lsb_release -a  # Should show 22.04 LTS Jammy

# Check resources
free -h         # Should be 8GB+ RAM
nproc           # Should be 4+ CPU
df -h           # Should have 80GB+ disk free

# Check ROS2 not installed yet
which ros2 || echo "ROS2 not installed - good, we will install"
```

If RAM <8GB, increase in UTM/VirtualBox settings and reboot VM.

---

## 1. Clone Your Repo (FULE-2.1)

```bash
cd ~
sudo apt update && sudo apt install -y git

# Clone your repo (you just pushed)
git clone https://github.com/Aster1145/FULE-2.1.git autonomous_maze_drone
cd autonomous_maze_drone

# Check files
ls -lh
# Should see: README_NIDAR.md, build_cpp.sh, src/fuel_exploration_cpp etc.

# Make scripts executable
chmod +x *.sh
chmod +x edge_deploy/*.sh
```

---

## 2. Install Dependencies (One Script - 20-30 mins)

```bash
cd ~/autonomous_maze_drone

# Option A: Use our install script (does ROS2 Humble + Gazebo Harmonic + PX4 deps)
./install_dependencies.sh
# This will take 20-30 mins, needs internet

# Option B: Manual if script fails (copy-paste)
sudo apt update
sudo apt install -y locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-base python3-colcon-common-extensions python3-rosdep
sudo apt install -y ros-humble-px4-msgs ros-humble-slam-toolbox ros-humble-nav2-bringup
sudo apt install -y gz-harmonic ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge ros-humble-ros-gzharmonic-image
sudo apt install -y ros-humble-depthai-ros ros-humble-depthai-ros-driver ros-humble-vision-msgs ros-humble-cv-bridge
sudo apt install -y build-essential cmake libopencv-dev libopencv-contrib-dev libeigen3-dev libyaml-cpp-dev libceres-dev libpcl-dev
sudo apt install -y libyaml-cpp-dev python3-pip
pip3 install depthai ultralytics transforms3d

# PX4 Autopilot (if not already)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
make px4_sitl gz_x500

# MicroXRCE Agent for PX4-ROS2 communication
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j4 && sudo make install && sudo ldconfig

# USB rules for OAK-D Lite + RPLidar
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# ROS2 setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash
sudo rosdep init || true
rosdep update
```

**After install, reboot VM**:
```bash
sudo reboot
```

---

## 3. Build C++ Fast Stack (Competition Version)

```bash
cd ~/autonomous_maze_drone
source /opt/ros/humble/setup.bash

# Install ROS deps
rosdep install --from-paths src --ignore-src -r -y --skip-keys="rplidar_ros depthai_ros_driver"

# Build C++ only (fast, -O3)
./build_cpp.sh

# Or manually:
colcon build --packages-select fuel_exploration_cpp offboard_control_cpp nodding_lidar_cpp air_mouse_cpp oak_d_lite_cpp \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native -std=c++17" --parallel-workers $(nproc)

# Build all packages (Python fallback + C++)
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
echo "source ~/autonomous_maze_drone/install/setup.bash" >> ~/.bashrc

# Verify
ros2 pkg list | grep -E "fuel|offboard|nodding|air_mouse|oak"
ros2 pkg executables fuel_exploration_cpp
# Should show: mapping_node, frontier_detector, planner_node, exploration_manager
```

**Expected build time**: 5-10 mins in VM.

---

## 4. Setup PX4 Models & NIDAR World

```bash
# Create Gazebo model folder
mkdir -p ~/.gz/models/

# Copy our custom drone (X500 with OAK-D Lite mount + nodding lidar)
cp -r ~/autonomous_maze_drone/src/drone_description/models/x500_maze_explorer ~/.gz/models/

# Copy NIDAR worlds to PX4
mkdir -p ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp ~/autonomous_maze_drone/src/drone_description/worlds/maze.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/maze.sdf
cp ~/autonomous_maze_drone/src/drone_description/worlds/nidar_air_mouse.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/nidar_air_mouse.sdf

# Copy custom PX4 airframe (4015)
cp ~/autonomous_maze_drone/src/px4_config/4015_gz_x500_maze_explorer ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4015_gz_x500_maze_explorer

# Test PX4 build
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

---

## 5. YOLOv8 ONNX Export for C++ Detector (Important for Speed)

Python YOLO is slow in VM. C++ uses ONNX via OpenCV DNN.

```bash
cd ~/autonomous_maze_drone
pip install ultralytics

# Export YOLOv8n to ONNX
yolo export model=yolov8n.pt format=onnx opset=12 dynamic=False simplify=True
# Creates yolov8n.onnx (~20MB)

mkdir -p src/air_mouse_cpp/config
cp yolov8n.onnx src/air_mouse_cpp/config/

# Verify
ls -lh src/air_mouse_cpp/config/yolov8n.onnx
```

For Jetson later, export TensorRT: `yolo export model=yolov8n.pt format=engine half=True`

---

## 6. Run Full Stack in VM - NIDAR AirMouse Simulation

### Method A: 5 Terminals (Recommended for learning)

**Terminal 1: MicroXRCE-DDS Agent (PX4-ROS2 bridge)**
```bash
MicroXRCEAgent udp4 -p 8888
# Keep running, should show "Running..."
```

**Terminal 2: PX4 SITL + Gazebo NIDAR World (15x15m, 6 survivors)**
```bash
cd ~/PX4-Autopilot

# For VM performance, use headless mode (no GUI, saves GPU)
export LIBGL_ALWAYS_SOFTWARE=1
HEADLESS=1 PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=nidar_air_mouse ./build/px4_sitl_default/bin/px4

# If you want GUI (slow in VM, may be black screen):
# PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=nidar_air_mouse ./build/px4_sitl_default/bin/px4
```
Wait until you see `INFO [commander] Ready for takeoff!`

**Terminal 3: ROS2 Bridges + Nodding Lidar C++**
```bash
source /opt/ros/humble/setup.bash
source ~/autonomous_maze_drone/install/setup.bash

# Gazebo to ROS2 bridges (lidar, cameras, IMU, clock)
ros2 launch drone_bringup gz_bridge.launch.py &

sleep 3

# Nodding lidar: oscillation + 2D->3D
ros2 run nodding_lidar_cpp oscillation_controller --ros-args -p min_angle_deg:=-45.0 -p max_angle_deg:=45.0 -p frequency_hz:=0.5 -p mode:=sinusoidal &
ros2 run nodding_lidar_cpp scan_to_3d --ros-args -p accumulate_scans:=20 -p min_range:=0.15 -p max_range:=12.0 &

# Check topics
ros2 topic list | grep -E "scan|servo|joint"
ros2 topic hz /scan
ros2 topic echo /servo_angle_cmd --once
```

**Terminal 4: C++ Autonomy Stack (FAST)**
```bash
source /opt/ros/humble/setup.bash
source ~/autonomous_maze_drone/install/setup.bash

# Mapping (2D grid from nodding lidar)
ros2 run fuel_exploration_cpp mapping_node --ros-args -p map_size:=15.0 -p resolution:=0.1 -p origin_x:=-7.5 -p origin_y:=-7.5 -p voxel_height:=3.0 &

# Frontier detector (FIS)
ros2 run fuel_exploration_cpp frontier_detector --ros-args -p min_frontier_size:=8 -p sensor_range:=4.0 -p info_gain_threshold:=10 &

# Planner (A* + B-spline)
ros2 run fuel_exploration_cpp planner_node --ros-args -p planning_height:=1.5 -p safety_margin:=0.3 &

# Survivor detector (YOLO ONNX)
ros2 run air_mouse_cpp survivor_detector --ros-args -p model:=/home/$USER/autonomous_maze_drone/src/air_mouse_cpp/config/yolov8n.onnx -p conf_thres:=0.4 -p max_survivors:=6 -p image_topic:=/camera/rgb/image_raw &

# Map generator for NIDAR scoring
ros2 run air_mouse_cpp map_generator &

# Failsafe (8ft height, geofence)
ros2 run air_mouse_cpp failsafe --ros-args -p max_height:=2.44 -p arena_size:=15.0 &

# Check
ros2 topic list | grep -E "map|frontier|trajectory|survivor|failsafe"
ros2 topic hz /map
```

**Terminal 5: Offboard Control C++ + Mission Manager**
```bash
source /opt/ros/humble/setup.bash
source ~/autonomous_maze_drone/install/setup.bash

# Offboard controller (arm, takeoff, follow trajectory)
ros2 run offboard_control_cpp offboard_controller --ros-args -p takeoff_height:=-1.5 &

# VIO bridge (VINS -> PX4)
ros2 run offboard_control_cpp vio_bridge &

sleep 3

# NIDAR Mission Manager (30 min timer, 6 survivors, return home)
ros2 run air_mouse_cpp mission_manager --ros-args -p arena_size:=15.0 -p max_mission_time:=1800.0 -p max_survivors:=6 -p home_x:=0.0 -p home_y:=0.0

# In another tab, check mission
# ros2 topic echo /mission/state
# ros2 topic echo /mission/survivors
```

**Expected Behavior**:
1. Drone arms, takes off to 1.5m (Terminal 5 shows "ARMING -> TAKEOFF")
2. Mapping builds 2D map from nodding lidar 3D cloud (check `ros2 topic echo /map --once`)
3. Frontier detector finds frontiers (unknown-adjacent cells) - `ros2 topic echo /frontiers`
4. Planner generates A* path + B-spline smoothing - `ros2 topic echo /trajectory`
5. Drone follows trajectory, explores maze
6. Survivor detector finds 6 persons (blue/green/red boxes in Gazebo) - `ros2 topic echo /person_detections`
7. Map generator saves `/tmp/nidar_2d_map_with_survivors.png` with orange circles
8. After exploration or 6 survivors, returns home (0,0) and lands
9. Final report at `/tmp/nidar_final_report.json`

### Method B: One-Liner Launch (C++)

```bash
source /opt/ros/humble/setup.bash
source ~/autonomous_maze_drone/install/setup.bash

# Launch all C++ nodes (except PX4 and Agent)
ros2 launch air_mouse_cpp air_mouse_cpp.launch.py

# This includes: fuel_cpp + offboard_cpp + nodding_cpp + mission_manager + survivor_detector + failsafe + map_generator
```

### Method C: Full System Launch (Python fallback if C++ fails)

```bash
ros2 launch drone_bringup nidar_air_mouse_sim.launch.py
```

---

## 7. Visualize in RViz2 (In VM)

```bash
# Terminal 6: RViz2 (slow in VM, use software rendering)
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source ~/autonomous_maze_drone/install/setup.bash
rviz2

# In RViz2, Add:
# - Map: topic /map
# - PointCloud2: /scan_3d_world (3D from nodding lidar)
# - MarkerArray: /frontiers, /survivor_markers, /planner/viz
# - Path: /trajectory
# - TF

# Or use Foxglove Studio (lighter than RViz2) if RViz2 slow
```

---

## 8. Check NIDAR Scoring Files

```bash
# After mission
ls -lh /tmp/nidar*
cat /tmp/nidar_survivors.json
cat /tmp/nidar_final_report.json
eog /tmp/nidar_2d_map_with_survivors.png  # or xdg-open

# Should show:
# - 2D map PGM + YAML for ROS nav
# - PNG with orange survivor circles + IDs for judges
# - JSON list of survivors x,y,conf
# - Final report with time, survivors found
```

These files are required for NIDAR AirMouse scoring.

---

## 9. Run Without Gazebo (Real Hardware Passthrough in VM)

Gazebo is heavy for MacBook VM. For final test, use real OAK-D Lite + RPLidar via USB passthrough.

**UTM USB Passthrough**:
1. UTM VM Settings → USB → Add Device → OAK-D Lite (03e7:2485) + RPLidar (10c4:ea60) + Arduino (1a86:7523)
2. In VM: `lsusb` should show 03e7, `ls /dev/ttyUSB*` should show RPLidar and Arduino

**Launch real hardware (no Gazebo, no PX4 SITL - use real Pixhawk if you have, or test mapping only)**:

```bash
# Terminal 1: OAK-D Lite driver
source ~/autonomous_maze_drone/install/setup.bash
ros2 launch oak_d_lite_driver oak_d_lite.launch.py &
# Check: ros2 topic hz /camera/left/image_raw

# Terminal 2: Nodding lidar real
ros2 launch nodding_lidar_cpp nodding_cpp.launch.py &
# Check: ros2 topic hz /scan, /scan_3d

# Terminal 3: C++ Autonomy
ros2 run fuel_exploration_cpp mapping_node &
ros2 run fuel_exploration_cpp frontier_detector &
ros2 run air_mouse_cpp survivor_detector &
rviz2  # Visualize /map, /scan_3d_world, /survivor_markers
```

This tests your C++ stack in VM with real sensors before Jetson.

---

## 10. Common Issues & Fixes in VM

| Issue | Fix |
|-------|-----|
| **Gazebo black screen** | `export LIBGL_ALWAYS_SOFTWARE=1`, use `HEADLESS=1` for PX4, or test without Gazebo using real sensors |
| **No /scan topic** | Check RPLidar USB passthrough, `sudo chmod 666 /dev/ttyUSB0`, `ros2 topic list` |
| **No /camera topics** | OAK-D Lite USB passthrough, check `lsusb`, add udev rule, `sudo apt install ros-humble-depthai-ros` |
| **VIO timeout failsafe** | VINS not running, for sim use `vio_bridge` dummy, for real need VINS-Fusion build |
| **Build fails Eigen** | `sudo apt install libeigen3-dev`, CMake has `-I/usr/include/eigen3` |
| **ONNX not found** | Export as in Step 5, check path `/home/$USER/.../yolov8n.onnx` |
| **VM slow, 100% CPU** | Use C++ nodes (not Python), increase VM RAM to 12GB, CPU 6, enable 3D accel, run headless Gazebo, close RViz2 |
| **PX4 not arming** | Check `MicroXRCEAgent` running, `ros2 topic echo /fmu/out/vehicle_status`, ensure VIO publishing |
| **Map not building** | Check `/scan_3d_world` publishing, `ros2 topic hz /map`, increase `accumulate_scans` to 10 |
| **YOLO no detections** | Lower `conf_thres` to 0.3, check `/camera/rgb/image_raw` with `ros2 run rqt_image_view rqt_image_view` |

---

## 11. Quick Commands Reference

```bash
# Build C++ fast
cd ~/autonomous_maze_drone
colcon build --packages-select fuel_exploration_cpp offboard_control_cpp nodding_lidar_cpp air_mouse_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source
source install/setup.bash

# List C++ executables
ros2 pkg executables fuel_exploration_cpp
ros2 pkg executables air_mouse_cpp

# Run mission manager alone (test FSM)
ros2 run air_mouse_cpp mission_manager --ros-args -p arena_size:=15.0 -p max_mission_time:=180.0

# Emergency stop (ground station button)
ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}" --once

# Save map manually
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: /tmp/maze_map}"

# Check mission state
ros2 topic echo /mission/state
ros2 topic echo /failsafe/reason
```

---

## 12. Next: Deploy to Jetson Orin Nano (After VM Test)

```bash
# On Jetson (Ubuntu 22.04 + JetPack 5.1.2)
# Copy src/*_cpp from VM to Jetson via scp
scp -r ~/autonomous_maze_drone/src/*_cpp nidar@jetson:~/ws/src/

ssh nidar@jetson
cd ~/ws
colcon build --packages-select fuel_exploration_cpp offboard_control_cpp nodding_lidar_cpp air_mouse_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native"
source install/setup.bash

# Max performance
sudo nvpmodel -m 0 && sudo jetson_clocks

# Export YOLO TensorRT for 30+ FPS
yolo export model=yolov8n.pt format=engine half=True
# Use engine in survivor_detector: model:=yolov8n.engine

# Launch real drone
ros2 launch air_mouse_cpp air_mouse_cpp.launch.py mode:=real
```

---

## Summary Checklist for VM

- [ ] Ubuntu 22.04 VM 8GB+ RAM, 4+ CPU, 80GB disk
- [ ] ROS2 Humble + Gazebo Harmonic + PX4 + depthai_ros installed
- [ ] Repo cloned https://github.com/Aster1145/FULE-2.1.git
- [ ] C++ build with -O3 success
- [ ] YOLO ONNX exported
- [ ] PX4 models/worlds copied
- [ ] MicroXRCEAgent running
- [ ] PX4 SITL NIDAR world running (headless)
- [ ] C++ nodes running: mapping, frontier, planner, survivor, failsafe, map_generator, offboard, mission_manager
- [ ] RViz2 shows /map, /scan_3d_world, /survivor_markers
- [ ] /tmp/nidar_2d_map_with_survivors.png generated with 6 survivors
- [ ] Mission returns home and lands

Good luck for NIDAR 2026 AirMouse! 🚁

For help, check: README_NIDAR.md, docs/MACBOOK_VM_SETUP.md, IMPLEMENTATION_GUIDE_VM_CPP.md
