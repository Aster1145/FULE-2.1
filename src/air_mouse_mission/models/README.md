# YOLO model weights

Place custom weights here, e.g. `best.pt`:

```
src/person_detection/models/best.pt
```

At runtime the perception nodes resolve weights in this order:

1. Absolute path (ROS `model` parameter).
2. `<share>/person_detection/models/<basename>` (installed from this dir).
3. `<share>/air_mouse_mission/models/<basename>`.
4. `~/.cache/nidar/models/<basename>` (auto-download cache).
5. Auto-download `yolov8n.pt` to the cache dir as a fallback.

Launch with a custom model:

```bash
ros2 run person_detection yolo_detector --ros-args -p model:=best.pt
ros2 run person_detection yolo_detector --ros-args -p model:=/path/to/best.pt
```
