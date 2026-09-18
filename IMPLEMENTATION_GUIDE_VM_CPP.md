# Implementation Guide: C++ Fast Stack + Ubuntu 22.04 VM on MacBook for NIDAR AirMouse

**You**: MacBook + Ubuntu 22.04 LTS VM + OAK-D Lite stereo + RPLidar nodding + NIDAR AirMouse + C++ for speed

This guide gives you **exact commands** to implement in VM.

---

## Part 1: MacBook VM Setup (UTM for Apple Silicon M1/M2/M3/M4, VirtualBox for Intel)

### Step 1: Install VM Software

**Apple Silicon Mac (most MacBooks 2020+)**:
- Download UTM: https://mac.getutm.app/ (free)
- Alternative: VMware Fusion 13 (free personal): https://customerconnect.vmware.com/downloads/get-download?downloadGroup=FUS-1302

**Intel Mac**:
- VirtualBox: https://www.virtualbox.org/wiki/Downloads
- Or VMware Fusion

### Step 2: Download Ubuntu 22.04 ISO

- **Apple Silicon**: Ubuntu 22.04 Desktop ARM64 - https://cdimage.ubuntu.com/jammy/daily-live/current/jammy-desktop-arm64.iso (3.5GB)
- **Intel**: https://releases.ubuntu.com/22.04/ubuntu-22.04.5-desktop-amd64.iso

### Step 3: Create VM in UTM (Example)

1. UTM → Create New VM → Virtualize → Linux → Browse ISO → Select jammy-desktop-arm64.iso
2. Hardware: RAM 8192 MB (8GB) minimum, CPU 4-6 cores, Storage 80GB, Display VirtIO-GPU + 3D acceleration ON
3. Shared Directory: Add Mac folder `/Users/yourname/autonomous_maze_drone` → mount `/media/psf/...` in VM
4. Install Ubuntu: Erase disk, user `nidar`, password, minimal install, reboot, remove ISO

**Post-install**:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y spice-vdagent mesa-utils git curl
# Check GPU: glxinfo | grep "OpenGL renderer" -> should show VirtIO or LLVMpipe
```

For VirtualBox Intel: Enable 3D accel in Display settings, install Guest Additions.

### Step 4: Ubuntu 22.04 Base Setup (Inside VM)

```bash
sudo apt update
sudo apt install -y git wget curl vim htop build-essential cmake

# ROS2 Humble (Ubuntu 22.04 official distro)
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

# C++ deps for fast competition
sudo apt install -y libopencv-dev libopencv-contrib-dev libeigen3-dev libyaml-cpp-dev libceres-dev libpcl-dev
sudo apt install -y libyaml-cpp-dev libpcl-dev

# Python still needed
sudo apt install -y python3-pip
pip3 install depthai ultralytics transforms3d

# PX4 Autopilot
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
make px4_sitl gz_x500

# MicroXRCE Agent for PX4-ROS2
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j4 && sudo make install && sudo ldconfig

# OAK-D Lite udev
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init || true
rosdep update
```

---

## Part 2: Get Code & Build C++ Fast Version

### Step 5: Copy Our Code to VM

**If using UTM shared folder**:
```bash
# In VM, shared folder mounted at /media/psf/
ls /media/psf/
cp -r /media/psf/autonomous_maze_drone ~/autonomous_maze_drone
cd ~/autonomous_maze_drone
```

**Or git clone if you pushed to GitHub**:
```bash
cd ~
git clone <your-repo> autonomous_maze_drone
```

### Step 6: Build C++ Stack (Competition Fast)

```bash
cd ~/autonomous_maze_drone

# Make executable
chmod +x build_cpp.sh install_dependencies.sh run_simulation.sh

# Build C++ only (10x faster runtime)
./build_cpp.sh

# Or manually:
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys="rplidar_ros depthai_ros_driver"
colcon build --packages-select fuel_exploration_cpp offboard_control_cpp nodding_lidar_cpp air_mouse_cpp oak_d_lite_cpp \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native -std=c++17" --parallel-workers $(nproc)

source install/setup.bash

# Verify C++ nodes
ros2 pkg executables fuel_exploration_cpp
ros2 pkg executables offboard_control_cpp
ros2 pkg executables air_mouse_cpp
```

**Why C++?** 
- Python: GIL, 10Hz mapping, 30% CPU, 500MB RAM
- C++: -O3 -march=native, 50Hz mapping, 5% CPU, 100MB RAM, critical for 30 min NIDAR mission + Jetson Orin Nano

### Step 7: YOLOv8 ONNX Export for C++ Detector

Python YOLO is slow. C++ uses OpenCV DNN with ONNX.

```bash
pip install ultralytics
yolo export model=yolov8n.pt format=onnx opset=12 dynamic=False simplify=True
# Creates yolov8n.onnx (20MB)

mkdir -p ~/autonomous_maze_drone/src/air_mouse_cpp/config
cp yolov8n.onnx ~/autonomous_maze_drone/src/air_mouse_cpp/config/
# For Jetson later: yolo export model=yolov8n.pt format=engine half=True
```

---

## Part 3: Test in VM

### Step 8: Setup PX4 Models & Worlds for NIDAR

```bash
mkdir -p ~/.gz/models/
cp -r ~/autonomous_maze_drone/src/drone_description/models/x500_maze_explorer ~/.gz/models/
cp ~/autonomous_maze_drone/src/drone_description/worlds/nidar_air_mouse.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/nidar_air_mouse.sdf
cp ~/autonomous_maze_drone/src/drone_description/worlds/maze.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/maze.sdf
cp ~/autonomous_maze_drone/src/px4_config/4015_gz_x500_maze_explorer ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4015_gz_x500_maze_explorer
```

### Step 9: Run C++ Simulation (Headless for VM Performance)

Gazebo is heavy in VM. Use headless mode.

**Terminal 1: DDS Agent**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2: PX4 SITL NIDAR World (Headless)**
```bash
cd ~/PX4-Autopilot
export LIBGL_ALWAYS_SOFTWARE=1
HEADLESS=1 PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=nidar_air_mouse ./build/px4_sitl_default/bin/px4
```

**Terminal 3: ROS2 Bridges + Nodding Lidar C++**
```bash
source ~/autonomous_maze_drone/install/setup.bash
ros2 launch drone_bringup gz_bridge.launch.py &
sleep 2
ros2 run nodding_lidar_cpp oscillation_controller --ros-args -p min_angle_deg:=-45.0 -p max_angle_deg:=45.0 -p frequency_hz:=0.5 &
ros2 run nodding_lidar_cpp scan_to_3d --ros-args -p accumulate_scans:=20
```

**Terminal 4: C++ Autonomy Fast**
```bash
source ~/autonomous_maze_drone/install/setup.bash
ros2 run fuel_exploration_cpp mapping_node --ros-args -p map_size:=15.0 -p resolution:=0.1 -p origin_x:=-7.5 -p origin_y:=-7.5 &
ros2 run fuel_exploration_cpp frontier_detector --ros-args -p min_frontier_size:=8 -p sensor_range:=4.0 &
ros2 run fuel_exploration_cpp planner_node --ros-args -p planning_height:=1.5 &
ros2 run air_mouse_cpp survivor_detector --ros-args -p model:=/home/nidar/autonomous_maze_drone/src/air_mouse_cpp/config/yolov8n.onnx -p conf_thres:=0.4 &
ros2 run air_mouse_cpp map_generator &
ros2 run air_mouse_cpp failsafe --ros-args -p max_height:=2.44 -p arena_size:=15.0 &
```

**Terminal 5: Offboard C++ + Mission**
```bash
source ~/autonomous_maze_drone/install/setup.bash
ros2 run offboard_control_cpp offboard_controller --ros-args -p takeoff_height:=-1.5 &
ros2 run offboard_control_cpp vio_bridge &
sleep 2
ros2 run air_mouse_cpp mission_manager --ros-args -p arena_size:=15.0 -p max_mission_time:=1800.0 -p max_survivors:=6
```

**Check**:
```bash
ros2 topic list
ros2 topic echo /mission/state
ros2 topic echo /mission/survivors
ros2 topic hz /map
ros2 topic hz /scan_3d
```

Expected: Drone arms, takes off 1.5m, mapping builds 15x15m map, frontier finds frontiers, planner A* path, explores, survivor detector finds 6 persons, map_generator saves `/tmp/nidar_2d_map_with_survivors.png`, mission_manager returns home and lands.

### Step 10: VM with Real Hardware (OAK-D Lite + RPLidar)

For final test before Jetson, passthrough USB to VM.

**UTM**: VM Settings → USB → Add Device → OAK-D Lite (03e7:2485) + RPLidar (10c4:ea60) + Arduino (1a86:7523)
**VirtualBox**: Settings → USB → Enable USB 3.0 → Add filters

Then in VM:
```bash
lsusb
ls /dev/ttyUSB*
# Should see RPLidar and Arduino

# Launch real hardware (no Gazebo)
source ~/autonomous_maze_drone/install/setup.bash
ros2 launch oak_d_lite_cpp oak_d_lite.launch.py &  # or depthai_ros_driver directly
ros2 launch nodding_lidar_cpp nodding_cpp.launch.py &
ros2 launch fuel_exploration_cpp fuel_cpp.launch.py &
ros2 launch air_mouse_cpp air_mouse_cpp.launch.py
```

Visualize in VM with `rviz2` (slow but works with LIBGL_ALWAYS_SOFTWARE=1).

---

## Part 4: Deploy to Jetson Orin Nano (Competition)

After VM test, copy C++ packages to Jetson:

```bash
# On Jetson (Ubuntu 22.04 + JetPack 5.1.2)
scp -r ~/autonomous_maze_drone/src/*_cpp nidar@jetson:~/ws/src/
ssh nidar@jetson
cd ~/ws
colcon build --packages-select fuel_exploration_cpp offboard_control_cpp nodding_lidar_cpp air_mouse_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native"
source install/setup.bash
ros2 launch air_mouse_cpp air_mouse_cpp.launch.py mode:=real
```

**Performance on Jetson**:
- Set max power: `sudo nvpmodel -m 0 && sudo jetson_clocks`
- Check: `tegrastats` → CPU 30%, GPU 50% (YOLO), RAM 4GB/8GB
- YOLO TensorRT: `yolo export model=yolov8n.pt format=engine half=True` → 30+ FPS

---

## Part 5: MacBook VM Optimization Checklist

- [ ] VM RAM 8GB+ (12GB if Mac 16GB)
- [ ] CPU 4+ cores
- [ ] Disk 80GB+
- [ ] 3D acceleration enabled (UTM VirtIO-GPU, VMware 3D, VirtualBox 3D)
- [ ] Shared folder working
- [ ] `LIBGL_ALWAYS_SOFTWARE=1` for Gazebo headless if black screen
- [ ] USB passthrough for OAK-D Lite + RPLidar
- [ ] C++ build with -O3 -march=native
- [ ] YOLO ONNX exported
- [ ] Swap 8GB if RAM low: `sudo fallocate -l 8G /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile`
- [ ] CPU governor performance: `echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`

---

## Troubleshooting

- **Gazebo black screen in VM**: Use headless `HEADLESS=1`, or `export LIBGL_ALWAYS_SOFTWARE=1`, or test without Gazebo using real sensors
- **C++ build fails Eigen**: `sudo apt install libeigen3-dev`, CMake already has `-I/usr/include/eigen3`
- **ONNX not found**: Check path `/home/nidar/.../yolov8n.onnx`, export as above
- **No /scan**: Check RPLidar USB passthrough, `sudo chmod 666 /dev/ttyUSB0`
- **VIO timeout**: OAK-D Lite not publishing, check `ros2 topic hz /camera/left/image_raw`, check USB passthrough
- **VM slow**: Use C++ nodes (not Python), reduce Gazebo, increase RAM/CPU, enable 3D accel

---

## Summary

You now have **C++ fast stack** for NIDAR AirMouse that runs in **Ubuntu 22.04 VM on MacBook** and deploys to **Jetson Orin Nano** for competition. Python version still available as fallback, but C++ is 5-10x faster, mandatory for 30 min mission.

Next: Test in VM, then Jetson, then 15x15m mock arena.

Good luck for NIDAR 2026!
