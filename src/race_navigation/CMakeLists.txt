cmake_minimum_required(VERSION 3.8)
project(race_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
