#!/bin/bash
# Jetson Orin Nano / NX setup for autonomous maze drone
# Ubuntu 22.04 + JetPack 5.1.2 + ROS2 Humble

set -e
echo "=== Jetson Edge Setup for Maze Drone ==="

# Check if Jetson
if [ ! -f "/etc/nv_tegra_release" ]; then
  echo "Warning: Not running on Jetson, but continuing for testing"
fi

# ROS2 Humble (if not installed)
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo "Installing ROS2 Humble on Jetson..."
  sudo apt update
  sudo apt install -y locales software-properties-common
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt install -y ros-humble-desktop ros-humble-ros-base python3-colcon-common-extensions
fi

# Jetson specific: install JetPack components
sudo apt install -y nvidia-jetpack
sudo apt install -y python3-pip libopenblas-dev

# Python deps with Jetson optimization
pip3 install --upgrade pip
pip3 install numpy opencv-python transforms3d scipy scikit-learn pyyaml

# PyTorch for Jetson (from NVIDIA)
# For JetPack 5.1.2, use torch 2.0.0+nv23.05
# wget https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
# pip3 install torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl

# Ultralytics YOLOv8 (CPU first, then TensorRT)
pip3 install ultralytics

# TensorRT for YOLO acceleration
sudo apt install -y tensorrt python3-libnvinfer-dev

# RPLidar udev rule
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# RealSense SDK (for VIO)
sudo apt install -y ros-humble-librealsense2* ros-humble-realsense2-*

# PX4 DDS Agent (for Pixhawk communication)
if [ ! -d "$HOME/Micro-XRCE-DDS-Agent" ]; then
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
  cd ~/Micro-XRCE-DDS-Agent && mkdir -p build && cd build && cmake .. && make -j4 && sudo make install && sudo ldconfig
fi

# VINS-Fusion deps
sudo apt install -y libceres-dev libgoogle-glog-dev libeigen3-dev

# Create workspace
mkdir -p ~/autonomous_maze_drone_ws/src
cd ~/autonomous_maze_drone_ws/src
# Copy from this repo if needed
# cp -r ~/autonomous_maze_drone/src/* .

# Performance tuning for Jetson
echo "Setting Jetson to MAXN power mode"
sudo nvpmodel -m 0 || true
sudo jetson_clocks || true

echo "=== Jetson setup complete ==="
echo "Next steps:"
echo "1. Build: cd ~/autonomous_maze_drone && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "2. Export YOLO to TensorRT: python3 src/person_detection/person_detection/yolo_export.py"
echo "3. Configure PX4 params (EKF2_AID_MASK=24, EKF2_HGT_REF=3)"
echo "4. Launch: ros2 launch drone_bringup full_system.launch.py mode:=real use_rplidar:=true use_realsense:=true"
