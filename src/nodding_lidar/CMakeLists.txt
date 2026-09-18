cmake_minimum_required(VERSION 3.8)
project(nodding_lidar)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/nodding_mechanism.py
  ${PROJECT_NAME}/scan_to_3d.py
  ${PROJECT_NAME}/oscillation_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
