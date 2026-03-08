cmake_minimum_required(VERSION 3.8)
project(seeker_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/gz_odom_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
