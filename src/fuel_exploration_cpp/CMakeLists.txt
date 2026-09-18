cmake_minimum_required(VERSION 3.8)
project(fuel_exploration_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(mapping_node src/mapping_node.cpp)
ament_target_dependencies(mapping_node rclcpp nav_msgs sensor_msgs geometry_msgs visualization_msgs tf2_ros)

add_executable(frontier_detector src/frontier_detector.cpp)
ament_target_dependencies(frontier_detector rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs)

add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs)

add_executable(exploration_manager src/exploration_manager.cpp)
ament_target_dependencies(exploration_manager rclcpp nav_msgs geometry_msgs std_msgs)

install(TARGETS
  mapping_node
  frontier_detector
  planner_node
  exploration_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
