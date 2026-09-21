#!/bin/bash
# setup_vm_env.sh - VM environment setup for FULE-2.1
# Ubuntu 22.04 ARM64 (UTM on Apple Silicon), ROS 2 Humble, Gazebo 11.
#
# What it does:
#   1. Exports VM-friendly display vars (X11 + software rendering).
#   2. Verifies / pins numpy<2.0.0 for python3-opencv compatibility.
#   3. Sources ROS 2 + workspace overlays.
#
# Usage:
#   source ./setup_vm_env.sh [workspace_path]
#   # or:
#   ./setup_vm_env.sh --check   # verify only, no sourcing
set -e

WORKSPACE="${1:-$HOME/autonomous_maze_drone}"
if [ "$1" = "--check" ]; then
  WORKSPACE="$HOME/autonomous_maze_drone"
  CHECK_ONLY=1
else
  CHECK_ONLY=0
fi

# --- 1. VM display fixes (Wayland/hardware-accel issues on UTM) ---
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
# Helpful extras for software GL in some Qt/Gazebo builds:
export QT_X11_NO_MITSHM=1
export __GLX_VENDOR_LIBRARY_NAME=mesa 2>/dev/null || true
echo "[setup_vm_env] QT_QPA_PLATFORM=$QT_QPA_PLATFORM LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"

# --- 2. NumPy compatibility check ---
echo "[setup_vm_env] Checking NumPy / OpenCV compatibility..."
if command -v python3 >/dev/null 2>&1; then
  NUMPY_VER=$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null || echo "missing")
  echo "[setup_vm_env] numpy version: $NUMPY_VER"
  MAJOR=$(echo "$NUMPY_VER" | cut -d. -f1)
  if [ "$NUMPY_VER" = "missing" ]; then
    echo "[setup_vm_env] WARNING: numpy not installed. Install with: pip3 install \"numpy<2.0.0\""
  elif [ "$MAJOR" = "2" ]; then
    echo "[setup_vm_env] ERROR: NumPy 2.x breaks python3-opencv (AttributeError: _ARRAY_API not found)."
    echo "[setup_vm_env] Fix: pip3 install --upgrade \"numpy<2.0.0\""
    if [ "$CHECK_ONLY" = "0" ]; then
      read -r -p "[setup_vm_env] Downgrade numpy now? [y/N] " ans
      if [ "$ans" = "y" ] || [ "$ans" = "Y" ]; then
        pip3 install --upgrade "numpy<2.0.0" -c "$(dirname "$0")/constraints.txt" 2>/dev/null \
          || pip3 install --upgrade "numpy<2.0.0"
      fi
    fi
  else
    echo "[setup_vm_env] NumPy OK (<2.0.0)."
  fi

  if python3 -c "import cv2" 2>/dev/null; then
    echo "[setup_vm_env] cv2 import OK."
  else
    echo "[setup_vm_env] WARNING: 'import cv2' failed. If you see '_ARRAY_API not found',"
    echo "  run: pip3 install --upgrade \"numpy<2.0.0\"  (then restart nodes)"
    python3 -c "import cv2" 2>&1 | head -n 5 || true
  fi
fi

# --- 3. Source ROS + workspace ---
if [ "$CHECK_ONLY" = "1" ]; then
  exit 0
fi

if [ -f "/opt/ros/humble/setup.bash" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  echo "[setup_vm_env] Sourced /opt/ros/humble/setup.bash"
else
  echo "[setup_vm_env] WARNING: /opt/ros/humble/setup.bash not found."
fi

if [ -f "$WORKSPACE/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE/install/setup.bash"
  echo "[setup_vm_env] Sourced $WORKSPACE/install/setup.bash"
else
  echo "[setup_vm_env] NOTE: workspace not built yet at $WORKSPACE."
  echo "  Build with: cd $WORKSPACE && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"
fi

echo "[setup_vm_env] Ready. Launch with:"
echo "  ros2 launch drone_bringup nidar_air_mouse_sim.launch.py"
