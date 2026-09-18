#!/usr/bin/env python3
"""
Export YOLOv8 to TensorRT for Jetson edge deployment
"""
import sys
try:
    from ultralytics import YOLO
    model = YOLO('yolov8n.pt')
    print("Exporting to TensorRT...")
    model.export(format='engine', half=True, device=0)
    print("Exported yolov8n.engine")
except Exception as e:
    print(f"Export failed: {e}")
    print("Make sure ultralytics and tensorrt are installed on Jetson")
