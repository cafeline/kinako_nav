cmake_minimum_required(VERSION 3.8)
project(kinako_nav_assets)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps2d maps3d waypoints regions rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
