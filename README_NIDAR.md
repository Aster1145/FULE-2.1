# NIDAR 2026 AirMouse - OAK-D Lite + Nodding RPLidar Implementation

**For NIDAR 2026-27 Mission 2: AirMouse - Autonomous GPS-Denied Indoor Search, Mapping & Survivor Localisation**

This is an updated version of the autonomous maze drone stack tailored for **OAK-D Lite stereo + RPLidar with oscillation mechanism** for NIDAR AirMouse 15x15m maze, 6 survivors, 30 min mission.

---

## NIDAR AirMouse Problem Statement (from nidar.org.in/missions)

- **Arena**: 15x15m indoor maze-like, corridors >=1m, rooms 2x2m, height 8ft (2.44m)
- **Launch Pad**: 2ft x 2ft (0.6096m), entry/exit same
- **Survivors**: Up to 6, must tag locations
- **Tasks**: Generate 2D map, locate survivors, return home, fully autonomous, no manual waypoints
- **Time**: Max 30 mins mission, 5 mins setup
- **Safety**: Emergency stop + failsafe mandatory
- **Prize**: Part of ₹67 Lakhs pool

---

## Your Hardware: OAK-D Lite + Nodding RPLidar

### OAK-D Lite (Luxonis)
- **RGB**: IMX214 13MP 81° DFOV, 1920x1080 @15Hz for YOLO
- **Stereo**: 2x OV7251 640x480 73° HFOV, 75mm baseline, 30Hz for VIO
- **IMU**: BMI270 6-axis, 200Hz
- **VPU**: Myriad X 4 TOPS (can run YOLO on-device)
- **ROS2**: `depthai_ros_driver` Humble, config in `oak_d_lite_driver/config/oak_d_lite_vio.yaml`
- **VIO**: VINS-Fusion ROS2 subscribing to left/right + IMU, or SpectacularAI SDK

### RPLidar 2D->3D Oscillation Mechanism
- **Hardware**: RPLidar A2M12 (360° 2D) + MG996R servo + Arduino Nano + 3D printed mount
- **Mechanism**: Servo nods lidar pitch -45° to +45° at 0.5Hz sinusoidal, 10Hz lidar = 20 scans per 3D sweep
- **ROS2**:
  - `oscillation_controller.py`: Generates sinusoidal angle, publishes `/servo_angle_cmd` (rad) + `/joint_states`
  - `nodding_mechanism.py`: Hardware interface to Arduino (serial "A90\n") or Dynamixel
  - `scan_to_3d.py`: Converts 2D scans + pitch into 3D pointcloud: `x' = x*cos(pitch)`, `z' = -x*sin(pitch)`, accumulates to `/scan_3d` and `/scan_3d_world`
- **Benefits**: Low-cost 3D lidar, covers floor to ceiling (8ft), still provides 2D projection for 2D map (NIDAR scoring)

See `docs/NIDAR_AIR_MOUSE_OAK_D_LITE.md` for detailed design, Arduino code in `edge_deploy/arduino_nodding_servo.ino`, mount in `src/drone_description/models/oak_d_lite_mount/`.

---

## Updated Software Stack

```
OAK-D Lite left/right + IMU -> VINS-Fusion -> /vins_estimator/camera_pose -> vio_bridge -> PX4 EKF2 (vision)
OAK-D Lite RGB -> survivor_detector (YOLOv8n) -> /person_detections -> mission_manager (NIDAR)

RPLidar + Servo -> /scan + /servo_angle -> scan_to_3d -> /scan_3d_world (3D) -> mapping_node -> /map (2D OccupancyGrid)
                                                               -> /scan_2d_projected

/map + /scan_3d_world -> frontier_detector (FIS) -> /exploration/goal -> planner_node (A* + B-spline) -> /trajectory -> offboard_controller -> PX4

mission_manager (FSM: SETUP->ARM->TAKEOFF->EXPLORE->GENERATE_MAP->RETURN_HOME->LAND) -> /mission/state, /mission/survivors, final report
failsafe (height 2.44m, geofence 15m, VIO/lidar timeout, battery, e-stop) -> /mission/abort
map_generator -> /tmp/nidar_2d_map.pgm/.yaml/.png with survivors for judges
```

**Packages**:
- `oak_d_lite_driver`: Config + launch for OAK-D Lite
- `nodding_lidar`: Oscillation controller + 2D->3D conversion + hardware interface
- `air_mouse_mission`: Mission manager + survivor detector + failsafe + map generator (NIDAR specific)
- `fuel_ros2_exploration`: FUEL-inspired mapping, frontier, planner (same as before)
- `offboard_control`: PX4 offboard + VIO bridge
- `drone_description`: NIDAR world `nidar_air_mouse.sdf` (15x15m, 6 survivors, launch pad)

---

## Quick Start - NIDAR Simulation

**Prerequisites**: Ubuntu 22.04, ROS2 Humble, Gazebo Harmonic, PX4, depthai_ros_driver

```bash
cd ~/autonomous_maze_drone
./install_dependencies.sh
sudo apt install ros-humble-depthai-ros ros-humble-depthai-ros-driver
pip install depthai ultralytics

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Copy NIDAR world
cp src/drone_description/worlds/nidar_air_mouse.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp -r src/drone_description/models/x500_maze_explorer ~/.gz/models/

# Run - 5 terminals (or ./run_simulation.sh with NIDAR world)
# T1: MicroXRCEAgent udp4 -p 8888
# T2: cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=nidar_air_mouse ./build/px4_sitl_default/bin/px4
# T3: ros2 launch drone_bringup gz_bridge.launch.py
# T4: ros2 launch nodding_lidar nodding_lidar.launch.py
# T5: ros2 launch drone_bringup autonomy.launch.py && ros2 launch air_mouse_mission air_mouse.launch.py
# T6: ros2 run offboard_control offboard_controller
```

**One-liner for NIDAR sim**:
```bash
ros2 launch drone_bringup nidar_air_mouse_sim.launch.py
```

**Check results**:
```bash
cat /tmp/nidar_survivors.json
cat /tmp/nidar_final_report.json
eog /tmp/nidar_2d_map_with_survivors.png
```

---

## Quick Start - Real Drone (Jetson Orin Nano + OAK-D Lite + Nodding RPLidar)

**Hardware Wiring**:
```
Jetson USB3 -> OAK-D Lite (USB-C)
Jetson USB -> RPLidar A2 (/dev/ttyUSB0)
Jetson USB -> Arduino Nano for servo (/dev/ttyUSB1)
Jetson UART (/dev/ttyTHS1) -> Pixhawk TELEM2 (921600 baud)
BEC 5V 4A -> Jetson + Servo (separate from Arduino 5V)
```

**Setup**:
```bash
./edge_deploy/jetson_setup.sh
sudo apt install ros-humble-depthai-ros
pip install depthai

# Arduino: Upload edge_deploy/arduino_nodding_servo.ino to Nano

# Build
colcon build
source install/setup.bash

# Launch full NIDAR mission
ros2 launch air_mouse_mission air_mouse.launch.py mode:=real
```

**Ground Station** (laptop):
```bash
rviz2  # Add /map, /scan_3d_world, /survivor_markers
ros2 topic echo /mission/state
ros2 topic echo /mission/survivors
# Emergency stop GUI: ros2 topic pub /emergency_stop std_msgs/Bool "{data: true}"
```

---

## NIDAR Scoring Files (Generated)

- `/tmp/nidar_2d_map.pgm` + `.yaml` - ROS map format for navigation
- `/tmp/nidar_2d_map_with_survivors.png` - Visual map with orange survivor circles + IDs for judges
- `/tmp/nidar_survivors.json` - List of survivors: id, x, y, conf, time
- `/tmp/nidar_final_report.json` - Mission stats: time, survivors found, complete
- `/tmp/person_detections.json` - Raw detections

These satisfy NIDAR requirements: 2D map, survivor tagging, return home, time.

---

## Failsafe & Emergency Stop (NIDAR Mandatory)

`failsafe.py` monitors:
- Max height 2.44m (8ft) -> abort if exceeded
- Geofence 15x15m -> abort if outside
- VIO timeout 2s, Lidar timeout 2s
- Low battery 14V (4S)
- Emergency stop button `/emergency_stop` (Bool) from ground station

On abort, publishes `/mission/abort` true and triggers LAND.

Emergency stop button: Use joystick or GUI publishing Bool.

---

## Competition Day Checklist

See `docs/NIDAR_AIR_MOUSE_OAK_D_LITE.md` - full checklist including OAK-D calibration, servo homing, VIO health, battery, launch pad, ground station RViz, PX4 params, etc.

---

## Differences from Previous Generic Stack

| Generic Stack | NIDAR AirMouse Stack |
|---------------|----------------------|
| RealSense D435i | OAK-D Lite (stereo OV7251 + RGB IMX214 + BMI270) |
| Fixed RPLidar 2D | Nodding RPLidar 2D->3D via servo oscillation |
| Generic maze | 15x15m NIDAR arena, 6 survivors, launch pad 2ft |
| Simple exploration | Mission manager with 30 min timer, return home, report |
| No failsafe | Failsafe with height, geofence, e-stop mandatory |
| Map only | Map + survivor tagging + final report for scoring |

---

## References

- NIDAR: https://www.nidar.org.in/missions (AirMouse)
- OAK-D Lite: https://shop.luxonis.com/products/oak-d-lite-1, depthai_ros_driver
- Nodding Lidar: Research "Coordinated Nodding of a 2D Lidar for Dense 3D Range Measurements"
- FUEL: https://github.com/HKUST-Aerial-Robotics/FUEL

---

## Next Steps

1. Build 3D printed mount for RPLidar + MG996R
2. Calibrate OAK-D Lite + IMU (Kalibr)
3. Tune VINS-Fusion for OAK-D Lite stereo (baseline 75mm)
4. Test nodding mechanism: 0.5Hz, -45° to +45°, check /scan_3d covers 8ft height
5. Full mission test in 15x15m mock arena
6. Prepare ground station GUI for judges (RViz + map + survivor list + timer + e-stop button)
