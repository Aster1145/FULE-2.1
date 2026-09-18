# NIDAR 2026 AirMouse - OAK-D Lite + Nodding RPLidar Implementation

## Mission Brief (from nidar.org.in/missions)

**AirMouse**: Autonomous GPS-Denied Indoor Search, Mapping & Survivor Localisation

- **Arena**: 15x15m indoor maze, corridors >=1m width, 8ft (2.44m) height clearance, rooms 2x2m
- **Launch Pad**: 2ft x 2ft (0.6096m), entry/exit same
- **Survivors**: Up to 6 (humans/dummies), must tag locations
- **Tasks**: Generate 2D map, locate survivors, exit, autonomous, no manual waypoints
- **Constraints**: Max 30 mins mission, 5 mins setup, emergency stop + failsafe mandatory
- **Scoring**: Map quality, survivor detection accuracy, time, return to home

## Our Hardware for NIDAR

### OAK-D Lite Camera (Stereo Vision)
- **Spec**: IMX214 13MP RGB 81° DFOV, 2x OV7251 640x480 mono stereo 73° HFOV, 75mm baseline, BMI270 IMU, Myriad X 4 TOPS
- **ROS2**: `depthai_ros_driver` (official Luxonis, Humble)
- **Usage**:
  - Left + Right mono 640x400 @30Hz for VIO (stereo + IMU)
  - RGB 1080P @15Hz for YOLO survivor detection
  - Depth aligned to RGB for 3D projection
  - IMU 200Hz for VINS-Fusion / SpectacularAI VIO

**VIO Options**:
1. **SpectacularAI** (recommended for OAK-D): `pip install depthai spectacularAI`, provides drift <1% distance, runs on host
2. **VINS-Fusion ROS2**: Subscribes to `/camera/left/image_raw`, `/camera/right/image_raw`, `/imu/data` - CPU heavy but accurate
3. **depthai_ros_driver + RTABMap**: Visual SLAM with loop closure

For NIDAR, we use **VINS-Fusion ROS2** with OAK-D Lite config (see `oak_d_lite_driver/config/oak_d_lite_vio.yaml`).

### RPLidar 2D->3D Nodding Mechanism
**Problem**: RPLidar A2/A3 is 2D 360° horizontal, but NIDAR maze needs 3D to see walls + survivors + floor.

**Solution**: Mount RPLidar on servo that oscillates pitch -45° to +45° (nodding lidar)

- **Hardware**: MG996R servo (10kg torque) + Arduino Nano + 3D printed bracket
- **Mechanism**: Servo rotates lidar around Y axis, 0° = horizontal, +45° = looking up to ceiling (8ft), -45° = looking down to floor
- **Scanning**: At 0.5Hz nodding, 10Hz lidar = 20 scans per sweep, each 360° horizontal at different pitch -> dense 3D pointcloud of room
- **ROS2**: 
  - `oscillation_controller.py`: Publishes sinusoidal angle `/servo_angle_cmd` (Float64 rad)
  - `nodding_mechanism.py`: Hardware interface to Arduino/Dynamixel
  - `scan_to_3d.py`: Converts each 2D scan + pitch angle into 3D points: `x' = x*cos(pitch)`, `z' = -x*sin(pitch)`, accumulates 20 scans into `/scan_3d` PointCloud2

**Benefits**:
- Low cost 3D lidar vs expensive 3D lidar
- Covers full room: floor, walls, ceiling, survivors
- 2D projection still available for 2D map (NIDAR requires 2D map)

**Arduino Code**: See `edge_deploy/arduino_nodding_servo.ino`

### Edge Compute: Jetson Orin Nano
- **Why**: 8GB RAM, 1024 CUDA cores, 32 Tensor cores, 6-core ARM, 7-15W
- **Tasks**:
  - OAK-D Lite driver (USB3)
  - VINS-Fusion VIO (CPU 2 cores)
  - Nodding lidar 3D accumulation (CPU)
  - FUEL exploration (mapping 10Hz, frontier 1Hz, planner 0.5Hz)
  - YOLOv8n survivor detection (GPU + TensorRT, 15Hz)
  - PX4 DDS agent

## Software Stack for NIDAR

```
OAK-D Lite (stereo left/right + IMU) -> VINS-Fusion ROS2 -> /vins_estimator/camera_pose (ENU)
                                      -> vio_bridge.py -> VehicleVisualOdometry (NED) -> PX4 EKF2

OAK-D Lite RGB -> survivor_detector.py (YOLOv8n) -> /person_detections (PoseStamped) -> mission_manager

RPLidar A2 + Servo (oscillation_controller) -> /scan + /servo_angle -> scan_to_3d.py -> /scan_3d_world (PointCloud2)
                                                                                          -> /scan_2d_projected -> mapping_node.py -> /map (OccupancyGrid)

Mapping: mapping_node (voxel + 2D) + slam_toolbox
Frontier: frontier_detector (FIS) -> /exploration/goal
Planner: planner_node (A* + B-spline) -> /trajectory -> offboard_controller -> PX4

Mission: mission_manager.py (FSM for NIDAR) + failsafe.py (height 2.44m, geofence 15m, e-stop)
Map: map_generator.py -> /tmp/nidar_2d_map.pgm/.yaml/.png with survivors
```

## Launch for NIDAR

**Simulation (Gazebo 15x15m maze, 6 survivors)**:
```bash
# Terminal 1: DDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: PX4 SITL with NIDAR world
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4015 PX4_GZ_MODEL=x500_maze_explorer PX4_GZ_WORLD=nidar_air_mouse ./build/px4_sitl_default/bin/px4

# Terminal 3: OAK-D Lite sim + nodding lidar + autonomy
source ~/autonomous_maze_drone/install/setup.bash
ros2 launch drone_bringup gz_bridge.launch.py  # sim bridges
ros2 launch nodding_lidar nodding_lidar.launch.py
ros2 launch drone_bringup autonomy.launch.py

# Terminal 4: Offboard + mission
ros2 run offboard_control offboard_controller
ros2 launch air_mouse_mission air_mouse.launch.py
```

**Real Drone (Jetson Orin Nano)**:
```bash
# Setup
./edge_deploy/jetson_setup.sh
# Wire: OAK-D Lite USB3, RPLidar USB, Arduino USB, Pixhawk TELEM2 UART

# Launch
source install/setup.bash
ros2 launch air_mouse_mission air_mouse.launch.py mode:=real
# Includes: oak_d_lite_driver, nodding_lidar, autonomy, mission_manager, failsafe, map_generator, offboard

# Ground station (laptop):
rviz2  # Add /map, /scan_3d_world, /survivor_markers, /final_2d_map
ros2 topic echo /mission/state
ros2 topic echo /mission/survivors
```

## NIDAR Scoring Implementation

- **2D Map**: `map_generator.py` saves `/tmp/nidar_2d_map.pgm` + `.yaml` + `.png` with survivors tagged as orange circles + IDs. Judges can view PNG.
- **Survivors**: `survivor_detector.py` + `mission_manager.py` save `/tmp/nidar_survivors.json` with x,y,conf,time, and `/tmp/nidar_final_report.json` with mission stats.
- **Return Home**: `mission_manager` publishes home goal (0,0) after exploration, checks distance <0.5m to land.
- **Emergency Stop**: `failsafe.py` subscribes to `/emergency_stop` (Bool) from ground station button (e.g., joystick or GUI), triggers abort and land.
- **Failsafe**: Height limit 2.44m (8ft), geofence 15m, VIO timeout 2s, lidar timeout 2s, low battery 14V, all trigger abort.

## OAK-D Lite Specific Config

- **USB Rules**: `echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules`
- **ROS2 Driver Install**: `sudo apt install ros-humble-depthai-ros ros-humble-depthai-ros-driver`
- **DepthAI SDK**: `pip install depthai`
- **SpectacularAI (optional VIO)**: `pip install spectacularAI` (requires license for commercial)

**Config file**: `oak_d_lite_driver/config/oak_d_lite_vio.yaml` - sets RGB 1080P 15Hz, stereo 400P 30Hz, IMU 200Hz, depth high accuracy.

## Nodding Lidar 3D Printed Mount

Design: RPLidar base plate + servo horn adapter + bearing support.
- STL files needed: `rplidar_mount.stl`, `servo_bracket.stl`
- Material: PETG or ABS for strength
- Weight: <100g total

Servo: MG996R 55g, 10kg/cm torque, 0.17s/60° speed, enough for RPLidar 190g.

## Testing for NIDAR

1. **VIO Test**: Move drone by hand in maze, check `/vins_estimator/camera_pose` drift <1%
2. **Nodding Test**: Run `oscillation_controller`, watch servo oscillate, check `/scan_3d` points cover floor to ceiling
3. **Mapping Test**: Walk with drone, check `/map` builds 15x15m maze correctly
4. **Survivor Test**: Place person in room, check YOLO detects and tags on map
5. **Mission Test**: Full autonomous from launch pad, explore, find 6 survivors, return, land, check final report

## Competition Day Checklist

- [ ] OAK-D Lite calibrated (factory calibration ok, but check)
- [ ] RPLidar nodding servo homed (90° center)
- [ ] VIO healthy (no timeout)
- [ ] Map clear
- [ ] Battery full 16.8V
- [ ] Launch pad 2ft x 2ft clear
- [ ] Emergency stop button working (ground station)
- [ ] Ground station RViz showing map + survivors
- [ ] PX4 params set (EKF2_AID_MASK=24, etc.)
- [ ] Max height param 2.44m set in failsafe
- [ ] Mission time 30 min timer visible
