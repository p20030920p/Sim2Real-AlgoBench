cmake_minimum_required(VERSION 3.8)
project(algo_nav2_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(algo_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Nav2 plugin library. One plugin class covers every registered algorithm;
# `algorithm` selects which. Registering several instances of this same class
# under different names is what lets Nav2 hold several algorithms loaded at once
# and pick one per goal via planner_id.
# ---------------------------------------------------------------------------
add_library(algo_nav2_plugins SHARED
  src/grid_planner_plugin.cpp
)

target_include_directories(algo_nav2_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # nav2_core is a header-only interface package: it exports include
  # directories but no CMake target, so it is consumed this way rather than
  # through target_link_libraries.
  ${nav2_core_INCLUDE_DIRS}
)

target_link_libraries(algo_nav2_plugins
  algo_core::algo_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
)

install(TARGETS algo_nav2_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

# Make the plugin discoverable by pluginlib.
pluginlib_export_plugin_description_file(nav2_core algo_plugins.xml)

ament_export_include_directories(include)
ament_export_libraries(algo_nav2_plugins)
ament_export_dependencies(
  algo_core
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2_ros
)

ament_package()
