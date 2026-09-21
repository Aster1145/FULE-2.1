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

# Gazebo Classic 11 (primary for nidar_air_mouse_sim.launch.py gzserver) +
# Gazebo Harmonic (fallback for ros_gz_sim stack). Install Classic first.
sudo apt install -y gazebo libgazebo-dev ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros || echo "gazebo_ros install skipped (check ROS apt source)"
sudo apt install -y gz-harmonic || echo "gz-harmonic install skipped"

# System OpenCV/NumPy for cv_bridge (prefer apt over pip to avoid ABI breaks)
sudo apt install -y python3-opencv python3-numpy

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
# CRITICAL (ARM64): pin numpy<2.0.0 FIRST. ROS 2 python3-opencv breaks with
# NumPy 2.x (AttributeError: _ARRAY_API not found). Install order matters.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
pip3 install --upgrade "numpy<2.0.0" -c "$SCRIPT_DIR/constraints.txt" 2>/dev/null || pip3 install --upgrade "numpy<2.0.0"
pip3 install ultralytics torch torchvision --extra-index-url https://download.pytorch.org/whl/cpu -c "$SCRIPT_DIR/constraints.txt" 2>/dev/null || pip3 install ultralytics torch torchvision --extra-index-url https://download.pytorch.org/whl/cpu
# NOTE: do NOT pip-install opencv-python on the ROS VM; use python3-opencv (apt).
pip3 install scipy pyyaml transforms3d scikit-learn -c "$SCRIPT_DIR/constraints.txt" 2>/dev/null || pip3 install scipy pyyaml transforms3d scikit-learn
# Full pinned set (same as above, reproducible):
pip3 install -r "$SCRIPT_DIR/requirements.txt" -c "$SCRIPT_DIR/constraints.txt" || true
python3 "$SCRIPT_DIR/scripts/check_numpy_compat.py" || echo "WARNING: NumPy/OpenCV check failed - run: pip3 install --upgrade \"numpy<2.0.0\""

# VM display defaults (UTM / Apple Silicon): X11 + software rendering.
echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc 2>/dev/null || true
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc 2>/dev/null || true
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1

# rosdep
sudo rosdep init || true
rosdep update

echo "=== Base installation complete ==="
echo "Next: Build workspace"
echo "cd ~/autonomous_maze_drone && source /opt/ros/humble/setup.bash && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"
