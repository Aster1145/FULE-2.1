#!/bin/bash
set -e

echo "=== Autonomous Maze Drone - Ubuntu 22.04 + ROS2 Humble Setup ==="
echo "Date: $(date)"
echo "User: $(whoami)"

# Update
sudo apt update
sudo apt install -y git wget curl python3-pip python3-rosdep python3-colcon-common-extensions

# ROS2 Humble
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo "Installing ROS2 Humble..."
  sudo apt install -y locales software-properties-common
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt update
  sudo apt install -y ros-humble-desktop ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge ros-humble-ros-gzharmonic-image ros-humble-ros-gzharmonic-sim
  sudo apt install -y ros-humble-slam-toolbox ros-humble-nav2-bringup ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-pcl-ros ros-humble-octomap ros-humble-octomap-ros ros-humble-tf2-ros ros-humble-image-transport
  sudo apt install -y ros-humble-px4-msgs
else
  echo "ROS2 Humble already installed"
fi

# Gazebo Harmonic
sudo apt install -y gz-harmonic

# PX4 deps
echo "Installing PX4 dependencies..."
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly libeigen3-dev libopencv-dev
pip3 install --user kconfiglib jsonschema future

# Micro XRCE DDS Agent
if [ ! -d "$HOME/Micro-XRCE-DDS-Agent" ]; then
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
  cd ~/Micro-XRCE-DDS-Agent
  mkdir -p build && cd build
  cmake .. && make -j$(nproc)
  sudo make install
  sudo ldconfig
fi

# PX4 Autopilot
if [ ! -d "$HOME/PX4-Autopilot" ]; then
  cd ~
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  cd PX4-Autopilot
  bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
  make px4_sitl gz_x500
fi

# VINS deps
sudo apt install -y libceres-dev libgoogle-glog-dev libeigen3-dev libopencv-contrib-dev libyaml-cpp-dev libpcl-dev

# Python deps
pip3 install ultralytics torch torchvision --extra-index-url https://download.pytorch.org/whl/cpu
pip3 install opencv-python numpy scipy pyyaml transforms3d scikit-learn

# rosdep
sudo rosdep init || true
rosdep update

echo "=== Base installation complete ==="
echo "Next: Build workspace"
echo "cd ~/autonomous_maze_drone && source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"
