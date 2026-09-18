# Hardware Setup - Real Drone

## Bill of Materials

| Component | Model | Purpose | Price |
|-----------|-------|---------|-------|
| Frame | Holybro X500 V2 | 500mm quad | $200 |
| FC | Pixhawk 6C | PX4 | $200 |
| Companion | Jetson Orin Nano 8GB | Edge compute | $499 |
| Stereo VIO | RealSense D435i | VIO + depth | $200 |
| Lidar | RPLidar A2M12 | 2D mapping | $350 |
| Camera | IMX219 8MP (or reuse D435i RGB) | YOLO | $30 |
| Motors | 4x 2216 920KV | Thrust | $100 |
| Props | 1045 | | $20 |
| Battery | 4S 5000mAh | Flight 15min | $60 |
| BEC | 5V 4A | Jetson power | $15 |

Total ~$1674

## Wiring Diagram

```
Battery 4S -> PDB -> ESCs (4x) -> Motors
             |
             -> 5V BEC -> Jetson Orin Nano (5V 4A)
             |
             -> Pixhawk 6C (power module)

Pixhawk TELEM2 (UART) -> Jetson UART (/dev/ttyTHS1, 921600 baud)
  TX -> RX, RX -> TX, GND -> GND

RPLidar USB -> Jetson USB3.0 (/dev/ttyUSB0)

RealSense D435i USB3 -> Jetson USB3.0 (use USB3 port, not USB2)

Jetson I2C not needed
```

## Jetson Orin Nano Mounting
- Mount on top plate with vibration dampers
- Ensure airflow for cooling (fan)
- Use 3M dual lock for easy removal

## Calibration

### 1. Camera-IMU for VIO
Use Kalibr:
```bash
# Record bag with chessboard moving
ros2 bag record /camera/left/image_raw /camera/right/image_raw /imu/data

# Kalibr calibration (on PC)
kalibr_calibrate_imu_camera --target aprilgrid.yaml --cam camchain.yaml --imu imu.yaml --bag vio_calib.bag
```

Copy result to `src/vins_fusion_ros2/config/real_sense_stereo_imu_config.yaml`:
```yaml
estimate_extrinsic: 2
extrinsic: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] # example
```

### 2. RPLidar to Base Link
Measure offset: RPLidar mounted 12cm above base, 0 forward.
In URDF: `<origin xyz="0 0 0.12" rpy="0 0 0"/>`
In PX4: not needed, lidar is for ROS mapping only

### 3. PX4 EKF2 VIO Setup
Via QGroundControl:
```
EKF2_AID_MASK = 24 (vision position + yaw)
EKF2_HGT_REF = 3 (vision)
EKF2_EV_DELAY = 100 ms (measure via timestamp diff)
EKF2_EV_POS_X = 0.15 (stereo cam forward)
EKF2_EV_POS_Y = 0.0
EKF2_EV_POS_Z = -0.05 (down is positive in NED? Check)
EKF2_GPS_CTRL = 0 (disable GPS indoors)
```

Test VIO: `listener vehicle_visual_odometry` in PX4 shell should show data.

### 4. YOLO Camera Calibration
For person projection, need camera intrinsics.
Use `camera_calibration` package:
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 --ros-args -r image:=/camera/rgb/image_raw
```
Save to `camera_info` topic.

## Safety Checklist Before Autonomous Flight

- [ ] Props removed for first tests
- [ ] VIO publishing at >20Hz, low drift
- [ ] RPLidar spinning, /scan publishing
- [ ] PX4 in Position mode holds position with VIO
- [ ] Offboard mode works: drone hovers at 1.5m when given setpoint
- [ ] Mapping builds map correctly in RViz
- [ ] Frontier detection finds frontiers
- [ ] Planner generates collision-free path
- [ ] YOLO detects persons at 3-5m
- [ ] Emergency RC switch configured (kill switch)
- [ ] Tethered flight first (rope)
- [ ] Indoor netted area

## Flight Test Procedure

1. **Manual Position Mode**: Fly manually, check VIO doesn't diverge
2. **Offboard Hover**: `ros2 run offboard_control offboard_controller` -> should hover
3. **Single Goal**: Publish `/exploration/goal` manually, check drone goes there
4. **Full Autonomy**: Launch `full_system.launch.py mode:=real`, monitor `/exploration/state`
5. **Maze**: Start at entrance, let drone explore, save map

## Edge Performance Tuning

On Jetson:
```bash
# Max performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Check resources
tegrastats
# Should show: CPU 30%, GPU 50% (YOLO), RAM 4GB/8GB

# If VINS slow: reduce max_cnt from 150 to 100, reduce image size to 640x400
# If YOLO slow: use yolov8n, TensorRT FP16, input 640
```
