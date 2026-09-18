#!/usr/bin/env python3
"""
exploration_manager.py - High-level FSM for autonomous maze exploration
Inspired by FUEL's exploration_manager

States:
- INIT
- TAKEOFF
- EXPLORING
- GOAL_REACHED
- RETURN_HOME
- LAND
- FINISHED

Manages mission, monitors progress, triggers replanning, saves map.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import json
import os
import time
import math

class ExplorationManager(Node):
    def __init__(self):
        super().__init__('exploration_manager')
        self.declare_parameter('exploration_timeout', 600.0)  # seconds
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('save_map_path', '/tmp/maze_map')

        self.timeout = self.get_parameter('exploration_timeout').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.save_path = self.get_parameter('save_map_path').value

        self.state = "INIT"
        self.start_time = time.time()
        self.last_goal_time = time.time()
        self.current_pose = None
        self.current_goal = None
        self.frontier_count = 0
        self.person_detections = []

        # Subs
        self.pose_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/exploration/goal', self.goal_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planner/path', self.path_callback, 10)
        self.person_sub = self.create_subscription(PoseStamped, '/person_detections', self.person_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Pubs
        self.state_pub = self.create_publisher(String, '/exploration/state', 10)
        self.return_pub = self.create_publisher(PoseStamped, '/exploration/goal', 10)
        self.mission_pub = self.create_publisher(Bool, '/mission/complete', 10)

        self.timer = self.create_timer(1.0, self.fsm_loop)

        self.get_logger().info(f"Exploration Manager started, home at {self.home_x}, {self.home_y}")

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.current_goal = msg
        self.last_goal_time = time.time()

    def path_callback(self, msg):
        pass

    def person_callback(self, msg):
        self.person_detections.append({
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'time': time.time()
        })
        self.get_logger().info(f"Person detected at {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}")

    def map_callback(self, msg):
        # Count unknown cells to check exploration progress
        unknown = sum(1 for v in msg.data if v == -1)
        total = len(msg.data)
        explored_ratio = 1 - unknown/total if total>0 else 0
        self.explored_ratio = explored_ratio

    def fsm_loop(self):
        # Publish state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        elapsed = time.time() - self.start_time

        if self.state == "INIT":
            self.get_logger().info("State INIT -> TAKEOFF")
            self.state = "TAKEOFF"
            # In real system, trigger takeoff via service
            self.takeoff_time = time.time()

        elif self.state == "TAKEOFF":
            # Wait 5 sec for takeoff
            if time.time() - self.takeoff_time > 5.0:
                self.get_logger().info("Takeoff complete -> EXPLORING")
                self.state = "EXPLORING"
                self.exploring_start = time.time()

        elif self.state == "EXPLORING":
            # Check timeout
            if elapsed > self.timeout:
                self.get_logger().info("Exploration timeout -> RETURN_HOME")
                self.state = "RETURN_HOME"
                return

            # Check if no frontiers (exploration complete)
            if hasattr(self, 'explored_ratio') and self.explored_ratio > 0.95:
                self.get_logger().info(f"Explored {self.explored_ratio*100:.1f}% -> RETURN_HOME")
                self.state = "RETURN_HOME"
                return

            # Check if stuck (no goal for long)
            if time.time() - self.last_goal_time > 10.0 and self.current_goal is None:
                # Try to publish home as goal to trigger replanning
                pass

            # Check goal reached
            if self.current_pose and self.current_goal:
                dx = self.current_pose.pose.position.x - self.current_goal.pose.position.x
                dy = self.current_pose.pose.position.y - self.current_goal.pose.position.y
                dist = math.hypot(dx, dy)
                if dist < 0.5:
                    self.get_logger().info(f"Goal reached, distance {dist:.2f}")
                    self.state = "GOAL_REACHED"

        elif self.state == "GOAL_REACHED":
            # Short pause, then continue exploring
            self.get_logger().info("Goal reached, continuing exploration")
            self.state = "EXPLORING"
            self.last_goal_time = time.time()

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
                    self.get_logger().info("Returned home -> LAND")
                    self.state = "LAND"

        elif self.state == "LAND":
            self.get_logger().info("Landing...")
            # Trigger landing via offboard controller
            # Save map and detections
            self.save_results()
            self.state = "FINISHED"

        elif self.state == "FINISHED":
            self.get_logger().info("Mission FINISHED")
            complete = Bool()
            complete.data = True
            self.mission_pub.publish(complete)

    def save_results(self):
        # Save person detections
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        detections_path = "/tmp/person_detections.json"
        with open(detections_path, 'w') as f:
            json.dump(self.person_detections, f, indent=2)
        self.get_logger().info(f"Saved {len(self.person_detections)} person detections to {detections_path}")

        # Save map via service call would be done externally
        # ros2 service call /slam_toolbox/save_map ...

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
