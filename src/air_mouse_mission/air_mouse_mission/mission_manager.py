#!/usr/bin/env python3
"""
mission_manager.py - NIDAR 2026 AirMouse Mission Manager
Mission Brief (from nidar.org.in/missions):
- Arena 15x15m, corridor width >=1m, height 8ft, room 2x2m
- Up to 6 survivors, entry/exit same (2ft x 2ft launch pad)
- Must generate 2D map, tag survivor locations, autonomous, no manual waypoint
- Max mission 30 mins, setup 5 mins
- Emergency stop / mission abort + failsafe required
- Scoring: map quality, survivors detected, time, return to home

This node is the high-level mission controller for AirMouse.
States:
IDLE -> SETUP -> ARM -> TAKEOFF -> EXPLORE_MAZE -> SEARCH_SURVIVORS -> GENERATE_MAP -> RETURN_HOME -> LAND -> COMPLETE
Handles failsafe, emergency stop, time monitoring.

Interfaces:
- Sub: /map, /person_detections, /offboard/pose, /emergency_stop
- Pub: /exploration/goal, /mission/state, /mission/time, /final_2d_map, /survivor_list
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Float32, Int32
import json
import time
import math
import os

class AirMouseMissionManager(Node):
    def __init__(self):
        super().__init__('air_mouse_mission_manager')
        self.declare_parameter('arena_size', 15.0)
        self.declare_parameter('max_mission_time', 1800.0)  # 30 mins
        self.declare_parameter('launch_pad_size', 0.6096)  # 2ft = 0.6096m
        self.declare_parameter('max_survivors', 6)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)

        self.arena_size = self.get_parameter('arena_size').value
        self.max_time = self.get_parameter('max_mission_time').value
        self.max_survivors = self.get_parameter('max_survivors').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value

        # Mission state
        self.state = "IDLE"
        self.start_time = None
        self.survivors_found = []  # list of dicts
        self.map = None
        self.current_pose = None
        self.emergency_stop = False
        self.mission_complete = False

        # Subs
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)
        self.survivor_sub = self.create_subscription(PoseStamped, '/person_detections', self.survivor_callback, 10)
        self.e_stop_sub = self.create_subscription(Bool, '/emergency_stop', self.e_stop_callback, 10)
        self.map_3d_sub = self.create_subscription(OccupancyGrid, '/scan_3d_world', self.map_3d_callback, 10)

        # Pubs
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.time_pub = self.create_publisher(Float32, '/mission/time_elapsed', 10)
        self.survivor_list_pub = self.create_publisher(String, '/mission/survivors', 10)
        self.return_pub = self.create_publisher(PoseStamped, '/exploration/goal', 10)
        self.abort_pub = self.create_publisher(Bool, '/mission/abort', 10)

        # Timer 1Hz FSM
        self.timer = self.create_timer(1.0, self.fsm_loop)

        self.get_logger().info(f"AirMouse Mission Manager: Arena {self.arena_size}m, Max {self.max_time}s, Home ({self.home_x},{self.home_y})")

    def map_callback(self, msg):
        self.map = msg

    def map_3d_callback(self, msg):
        pass

    def pose_callback(self, msg):
        self.current_pose = msg

    def survivor_callback(self, msg):
        # Check if new survivor (distance >1m from existing)
        new = True
        for s in self.survivors_found:
            dx = s['x'] - msg.pose.position.x
            dy = s['y'] - msg.pose.position.y
            if math.hypot(dx, dy) < 1.0:
                new = False
                break
        if new and len(self.survivors_found) < self.max_survivors:
            entry = {
                'id': len(self.survivors_found)+1,
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'time': time.time() - self.start_time if self.start_time else 0
            }
            self.survivors_found.append(entry)
            self.get_logger().info(f"Survivor {entry['id']} found at {entry['x']:.2f}, {entry['y']:.2f} - Total {len(self.survivors_found)}/{self.max_survivors}")

            # Publish list
            self.publish_survivors()

    def e_stop_callback(self, msg):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP TRIGGERED!")
            self.state = "ABORT"

    def publish_survivors(self):
        msg = String()
        msg.data = json.dumps(self.survivors_found)
        self.survivor_list_pub.publish(msg)

        # Save to file for NIDAR scoring
        with open('/tmp/nidar_survivors.json', 'w') as f:
            json.dump(self.survivors_found, f, indent=2)

    def fsm_loop(self):
        # Publish state and time
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        if self.start_time is not None:
            elapsed = time.time() - self.start_time
            time_msg = Float32()
            time_msg.data = elapsed
            self.time_pub.publish(time_msg)

            # Check max time
            if elapsed > self.max_time:
                self.get_logger().warn(f"Max mission time {self.max_time}s exceeded, returning home")
                self.state = "RETURN_HOME"

        # FSM
        if self.state == "IDLE":
            self.get_logger().info("IDLE -> SETUP (waiting for arm command)")
            self.state = "SETUP"
            self.setup_time = time.time()

        elif self.state == "SETUP":
            # 5 min setup time allowed, we simulate 5 sec
            if time.time() - self.setup_time > 5.0:
                self.get_logger().info("SETUP complete -> ARM")
                self.state = "ARM"

        elif self.state == "ARM":
            self.get_logger().info("ARM -> TAKEOFF")
            self.state = "TAKEOFF"
            self.takeoff_time = time.time()
            self.start_time = time.time()

        elif self.state == "TAKEOFF":
            if time.time() - self.takeoff_time > 5.0:
                self.get_logger().info("TAKEOFF complete -> EXPLORE_MAZE")
                self.state = "EXPLORE_MAZE"
                self.explore_start = time.time()

        elif self.state == "EXPLORE_MAZE":
            # Exploration managed by fuel_ros2_exploration
            # Check if exploration complete or all survivors found
            if len(self.survivors_found) >= self.max_survivors:
                self.get_logger().info(f"All {self.max_survivors} survivors found! -> GENERATE_MAP")
                self.state = "GENERATE_MAP"
            elif self.map is not None:
                # Check explored ratio
                unknown = sum(1 for v in self.map.data if v == -1)
                total = len(self.map.data)
                explored = 1 - unknown/total if total>0 else 0
                if explored > 0.90:
                    self.get_logger().info(f"Explored {explored*100:.1f}% -> GENERATE_MAP")
                    self.state = "GENERATE_MAP"

        elif self.state == "GENERATE_MAP":
            self.get_logger().info("Generating final 2D map for NIDAR scoring...")
            # Trigger map save via service or node
            self.save_final_map()
            self.state = "RETURN_HOME"

        elif self.state == "RETURN_HOME":
            # Publish home goal
            home_goal = PoseStamped()
            home_goal.header.frame_id = "map"
            home_goal.header.stamp = self.get_clock().now().to_msg()
            home_goal.pose.position.x = self.home_x
            home_goal.pose.position.y = self.home_y
            home_goal.pose.position.z = 1.5
            home_goal.pose.orientation.w = 1.0
            self.return_pub.publish(home_goal)

            if self.current_pose:
                dx = self.current_pose.pose.position.x - self.home_x
                dy = self.current_pose.pose.position.y - self.home_y
                if math.hypot(dx, dy) < 0.5:
                    self.get_logger().info("Returned to launch pad -> LAND")
                    self.state = "LAND"

        elif self.state == "LAND":
            self.get_logger().info("Landing on launch pad (2ft x 2ft)")
            self.state = "COMPLETE"
            self.mission_complete = True
            self.save_final_report()

        elif self.state == "COMPLETE":
            self.get_logger().info(f"MISSION COMPLETE! Survivors {len(self.survivors_found)}/{self.max_survivors}, Time {time.time()-self.start_time:.1f}s")

        elif self.state == "ABORT":
            self.get_logger().error("MISSION ABORTED - Emergency stop or failsafe")
            abort_msg = Bool()
            abort_msg.data = True
            self.abort_pub.publish(abort_msg)
            # Trigger landing
            self.state = "LAND"

    def save_final_map(self):
        # Save map as image and yaml for NIDAR
        if self.map is None:
            return
        # The actual saving is done by maze_mapping/map_saver, we just ensure files exist
        self.get_logger().info("Final map should be at /tmp/maze_map_with_persons.png and /tmp/maze_map.pgm")

    def save_final_report(self):
        report = {
            'mission': 'NIDAR 2026 AirMouse',
            'arena_size': self.arena_size,
            'max_time': self.max_time,
            'actual_time': time.time() - self.start_time if self.start_time else 0,
            'survivors_found': len(self.survivors_found),
            'survivors': self.survivors_found,
            'home': {'x': self.home_x, 'y': self.home_y},
            'complete': self.mission_complete
        }
        with open('/tmp/nidar_final_report.json', 'w') as f:
            json.dump(report, f, indent=2)
        self.get_logger().info(f"Final report saved to /tmp/nidar_final_report.json: {report}")

def main(args=None):
    rclpy.init(args=args)
    node = AirMouseMissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
