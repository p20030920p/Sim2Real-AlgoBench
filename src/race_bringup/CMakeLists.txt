cmake_minimum_required(VERSION 3.8)
project(race_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  rviz_compat
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/rviz_compat
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
