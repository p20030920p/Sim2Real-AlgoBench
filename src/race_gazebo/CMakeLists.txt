cmake_minimum_required(VERSION 3.8)
project(race_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
