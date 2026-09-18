cmake_minimum_required(VERSION 3.8)
project(fuel_ros2_exploration)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/mapping_node.py
  ${PROJECT_NAME}/frontier_detector.py
  ${PROJECT_NAME}/planner_node.py
  ${PROJECT_NAME}/exploration_manager.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
