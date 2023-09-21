cmake_minimum_required(VERSION 3.8)
project(weed_vision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})


ament_package()

install(
  DIRECTORY launch rviz bag models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/scripts/model.py
  DESTINATION lib/${PROJECT_NAME}
)
