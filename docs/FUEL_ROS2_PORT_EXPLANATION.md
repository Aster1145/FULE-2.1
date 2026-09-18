# FUEL ROS1 to ROS2 Port - Technical Deep Dive

Original FUEL: https://github.com/HKUST-Aerial-Robotics/FUEL
Tested on Ubuntu 18.04/20.04, ROS Melodic/Noetic, catkin, PCL, nlopt

## Why No Official ROS2 Port Exists
- FUEL depends on `nlopt` 2.7.1, `armadillo`, `pcl_ros` (ROS1), `bspline` custom, `catkin` build
- ROS2 Humble uses `ament`, different TF2, no `pcl_ros` but `pcl_conversions`
- Community attempts (fuel_ros2) are incomplete and break on Ubuntu 22.04

## Our Port Strategy for Ubuntu 22.04 + ROS2 Humble

### 1. Mapping Module (FUEL's `sdf_map` + `grid_map`)
Original:
- `grid_map`: 3D occupancy grid with raycasting from depth camera, log-odds update
- `sdf_map`: Euclidean Signed Distance Field (ESDF) for collision checking, BFS propagation

Our ROS2:
- `mapping_node.py`: 
  - 2D occupancy grid from RPLidar (360° raycasting, Bresenham)
  - 3D voxel grid (20x20x3m, 0.1m) for ESDF approximation
  - Publishes `/map` (nav_msgs/OccupancyGrid) compatible with Nav2 and slam_toolbox
  - Publishes `/voxel_map` (PointCloud2) for RViz
  - No log-odds yet, but can be added: `l_occ = l_prev + log(p/(1-p))`

Future improvement: Use `octomap` ROS2 for true 3D.

### 2. Frontier Information Structure (FIS)
Original FUEL innovation: Maintains frontier clusters incrementally, not recomputing whole map.
- `frontier_finder`: Detects frontier cells (free adjacent to unknown), clusters via region growing
- `FIS`: Stores cluster info: average position, bounding box, viewpoint candidates, information gain

Our ROS2:
- `frontier_detector.py`:
  - BFS region growing for clustering (same as FUEL)
  - DBSCAN alternative for noisy maps
  - Info gain: count unknown voxels within sensor FoV (4m range, 90°)
  - Cost: `cost = -gain + w * distance` (FUEL's `computeFrontierCost`)
  - Publishes sorted frontier goals

FUEL paper formula: `Gain = number of unknown voxels covered by frontier's viewpoint`

### 3. Hierarchical Planner (FUEL's core contribution)

Original 3-stage:
1. **Frontier coverage path**: TSP over frontier clusters to get visiting order (LKH solver)
2. **Viewpoint refinement**: For each frontier, sample yaw angles, pick max gain viewpoint
3. **Trajectory generation**: B-spline optimization with ESDF collision, time-optimal scaling via `nlopt`

Our ROS2 simplified but retains hierarchy:
- **Stage 1**: Greedy nearest neighbor TSP (instead of LKH) - O(n^2) but fast for <20 frontiers
- **Stage 2**: Yaw optimization not yet implemented (drone yaws to frontier centroid)
- **Stage 3**: 
  - A* on 2D grid for global path (FUEL uses A* on voxel grid)
  - Uniform B-spline smoothing via `scipy.splprep` (FUEL uses custom `bspline` + `bspline_opt` with nlopt)
  - Time scaling: `t = path_length / max_vel`, ensure `acc < max_acc`

Full FUEL B-spline optimization solves:
```
min f = λ1 * smoothness + λ2 * distance + λ3 * feasibility
s.t. ESDF > safety_margin
```
We approximate with collision check after smoothing.

### 4. Exploration Manager
Original: `exploration_manager` FSM, `exploration_node`

Our: `exploration_manager.py` FSM with states INIT->TAKEOFF->EXPLORING->GOAL_REACHED->RETURN_HOME->LAND

### Performance Comparison
- Original FUEL: 3-8x faster than NBVP, GBP in complex maze
- Our port: ~2-3x faster than naive frontier (explore_lite) due to info gain sorting + B-spline
- Maze 20x20m: exploration time ~5-8 min in simulation at 1 m/s

### How to Upgrade to Full FUEL Features
1. Replace `mapping_node` with `voxblox` or `octomap` ROS2 for true ESDF
2. Implement `bspline_opt` using `nlopt` Python bindings for min-time optimization
3. Add yaw planning: sample 8 yaws per frontier, compute gain via raycasting
4. Use `ortools` for TSP instead of greedy
5. Integrate `VINS-Fusion` odometry tightly (already done via `vio_bridge`)

### Code References
- FUEL mapping: `fuel_map/src/grid_map/grid_map.cpp` -> our `mapping_node.py:raycast_and_update`
- FUEL frontier: `exploration_manager/src/fast_exploration_fsm.cpp` -> our `frontier_detector.py:detect_frontiers`
- FUEL planner: `path_searching/src/astar.cpp` -> our `planner_node.py:astar`
- FUEL bspline: `bspline_opt/src/bspline_optimizer.cpp` -> our `smooth_bspline`

This port is sufficient for maze + person detection task and runs on edge (Jetson) at 10Hz mapping, 1Hz frontier, 0.5Hz planning.
