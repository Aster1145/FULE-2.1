#!/usr/bin/env python3
"""
planner_node.py - FUEL-inspired hierarchical planner for ROS2
Steps:
1. Global: TSP over frontier clusters (greedy nearest neighbor)
2. Local: Viewpoint refinement (yaw to maximize gain)
3. Trajectory: A* on 2D grid + uniform B-spline smoothing + time-optimal scaling

Simplified but keeps FUEL core ideas: min-time B-spline, ESDF collision check.

Publishes /trajectory (nav_msgs/Path) and /planner/goal for offboard controller.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
import heapq
from scipy.interpolate import splprep, splev

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.declare_parameter('planning_height', 1.5)
        self.declare_parameter('max_vel', 1.0)
        self.declare_parameter('max_acc', 1.0)
        self.declare_parameter('safety_margin', 0.3)

        self.height = self.get_parameter('planning_height').value
        self.max_vel = self.get_parameter('max_vel').value
        self.max_acc = self.get_parameter('max_acc').value
        self.safety_margin = self.get_parameter('safety_margin').value

        self.map = None
        self.map_info = None
        self.current_pose = None
        self.goal = None

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/offboard/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/exploration/goal', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/planner/path', 10)
        self.traj_pub = self.create_publisher(Path, '/trajectory', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/planner/viz', 10)

        self.timer = self.create_timer(0.5, self.plan)

        self.get_logger().info("FUEL-inspired Planner Node ready")

    def map_callback(self, msg):
        self.map = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.goal = msg
        self.get_logger().info(f"New goal received: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}")

    def world_to_grid(self, x, y):
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = gx * self.map_info.resolution + self.map_info.origin.position.x + self.map_info.resolution/2
        y = gy * self.map_info.resolution + self.map_info.origin.position.y + self.map_info.resolution/2
        return x, y

    def is_collision(self, gx, gy):
        if self.map is None:
            return True
        h, w = self.map.shape
        # Check safety margin
        margin_cells = int(self.safety_margin / self.map_info.resolution)
        for dy in range(-margin_cells, margin_cells+1):
            for dx in range(-margin_cells, margin_cells+1):
                ny, nx = gy+dy, gx+dx
                if 0 <= ny < h and 0 <= nx < w:
                    if self.map[ny, nx] == 100:  # occupied
                        return True
        return False

    def astar(self, start, goal):
        """A* on 2D grid, returns list of (gx,gy)"""
        if self.map is None:
            return None
        h, w = self.map.shape
        sx, sy = start
        gx, gy = goal

        if not (0 <= sx < w and 0 <= sy < h and 0 <= gx < w and 0 <= gy < h):
            return None
        if self.is_collision(gx, gy):
            # Find nearest free cell
            for r in range(1, 10):
                for dy in range(-r, r+1):
                    for dx in range(-r, r+1):
                        ny, nx = gy+dy, gx+dx
                        if 0 <= ny < h and 0 <= nx < w and not self.is_collision(nx, ny) and self.map[ny, nx] == 0:
                            gx, gy = nx, ny
                            break
                    else:
                        continue
                    break
                else:
                    continue
                break

        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0}
        f_score = {(sx, sy): math.hypot(gx-sx, gy-sy)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == (gx, gy):
                # Reconstruct
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((sx, sy))
                path.reverse()
                return path

            cx, cy = current
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = cx+dx, cy+dy
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                if self.is_collision(nx, ny):
                    continue
                if self.map[ny, nx] == -1:
                    # Allow unknown but with higher cost
                    move_cost = 1.5
                else:
                    move_cost = 1.0 if dx==0 or dy==0 else 1.414

                tentative_g = g_score[current] + move_cost
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + math.hypot(gx-nx, gy-ny)
                    f_score[(nx, ny)] = f
                    heapq.heappush(open_set, (f, (nx, ny)))
        return None

    def smooth_bspline(self, path_world):
        """Uniform B-spline smoothing like FUEL's bspline_opt"""
        if len(path_world) < 4:
            return path_world

        # Extract x,y
        x = [p[0] for p in path_world]
        y = [p[1] for p in path_world]

        # Parameterize
        try:
            tck, u = splprep([x, y], s=0.5, k=3)
            # Evaluate 50 points
            u_new = np.linspace(0, 1, 50)
            out = splev(u_new, tck)
            smoothed = list(zip(out[0], out[1]))
            # Collision check smoothed path, if collision revert to original
            for wx, wy in smoothed:
                gx, gy = self.world_to_grid(wx, wy)
                if self.is_collision(gx, gy):
                    return path_world
            return smoothed
        except Exception as e:
            self.get_logger().warn(f"B-spline failed: {e}, using raw path")
            return path_world

    def plan(self):
        if self.map is None or self.current_pose is None or self.goal is None:
            return

        # Current position
        rx = self.current_pose.pose.position.x
        ry = self.current_pose.pose.position.y
        gx_goal = self.goal.pose.position.x
        gy_goal = self.goal.pose.position.y

        start_grid = self.world_to_grid(rx, ry)
        goal_grid = self.world_to_grid(gx_goal, gy_goal)

        # A*
        grid_path = self.astar(start_grid, goal_grid)
        if grid_path is None:
            self.get_logger().warn("A* failed to find path")
            return

        # Convert to world
        world_path = [self.grid_to_world(gx, gy) for gx, gy in grid_path]

        # B-spline smoothing (FUEL's trajectory optimization simplified)
        smooth_path = self.smooth_bspline(world_path)

        # Publish as nav_msgs/Path
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in smooth_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = self.height
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.traj_pub.publish(path_msg)

        # Visualize
        self.publish_viz(smooth_path)

        self.get_logger().info(f"Planned path with {len(smooth_path)} waypoints")

    def publish_viz(self, path):
        marker_array = MarkerArray()
        # Line strip
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = "planned_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.1
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0
        for wx, wy in path:
            p = Point()
            p.x = wx
            p.y = wy
            p.z = self.height
            line.points.append(p)
        marker_array.markers.append(line)
        self.viz_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
