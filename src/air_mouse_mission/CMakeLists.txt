cmake_minimum_required(VERSION 3.8)
project(air_mouse_mission)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/mission_manager.py
  ${PROJECT_NAME}/survivor_detector.py
  ${PROJECT_NAME}/map_generator.py
  ${PROJECT_NAME}/failsafe.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
