cmake_minimum_required(VERSION 3.8)
project(air_mouse_cpp)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -march=native)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(mission_manager src/mission_manager.cpp)
ament_target_dependencies(mission_manager rclcpp nav_msgs geometry_msgs std_msgs)

add_executable(failsafe src/failsafe.cpp)
ament_target_dependencies(failsafe rclcpp geometry_msgs std_msgs sensor_msgs px4_msgs)

add_executable(map_generator src/map_generator.cpp)
ament_target_dependencies(map_generator rclcpp nav_msgs visualization_msgs)

add_executable(survivor_detector src/survivor_detector.cpp)
ament_target_dependencies(survivor_detector rclcpp sensor_msgs geometry_msgs visualization_msgs)

install(TARGETS mission_manager failsafe map_generator survivor_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
