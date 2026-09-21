cmake_minimum_required(VERSION 3.8)
project(person_detection)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/yolo_detector.py
  ${PROJECT_NAME}/yolo_export.py
  DESTINATION lib/${PROJECT_NAME}
)
# YOLO weights live in models/ (best.pt etc.) and are resolved at runtime
# via ament_index_python. Installed so share/person_detection/models works.
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
ament_package()
