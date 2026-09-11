cmake_minimum_required(VERSION 3.8)
project(algo_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# The registry helper is a plain Python module so that both the launch files and
# any node in race_control can import it to resolve an algorithm index.
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake_python rclpy)

ament_package()
