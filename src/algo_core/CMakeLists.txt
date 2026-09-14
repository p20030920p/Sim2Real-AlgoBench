cmake_minimum_required(VERSION 3.8)
project(algo_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# The algorithm library itself. Deliberately depends on nothing but the C++
# standard library: no ROS, no costmap types, no parameter framework. That is
# what lets it be unit tested offline and reused outside Nav2.
#
# SHARED, not STATIC, on purpose: algorithms self-register from static
# initialisers in their own translation units, and a static archive lets the
# linker drop unreferenced objects — which silently empties the registry.
# ---------------------------------------------------------------------------
add_library(algo_core SHARED
  src/registry.cpp
  src/graph/astar.cpp
  src/graph/jps.cpp
  src/graph/theta_star.cpp
  src/graph/d_star_lite.cpp
)
target_include_directories(algo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS algo_core
  EXPORT export_algo_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_algo_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(algo_core)

# ---------------------------------------------------------------------------
# ROS-free self test: builds a synthetic map, runs every registered algorithm
# and checks the invariants (path found, path connects start to goal, path is
# collision free). Runnable with `colcon test` or directly.
# ---------------------------------------------------------------------------
add_executable(algo_core_selftest test/selftest.cpp)
target_link_libraries(algo_core_selftest algo_core)
install(TARGETS algo_core_selftest DESTINATION lib/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Dumps what each algorithm did on a real map (expansion order and path), so the
# README demo is rendered from the shipped code rather than from a separate
# implementation that could drift away from it.
# ---------------------------------------------------------------------------
add_executable(algo_plan_dump tools/plan_dump.cpp)
target_link_libraries(algo_plan_dump algo_core)
install(TARGETS algo_plan_dump DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    ament_lint_auto_find_test_dependencies()
  endif()

  add_test(NAME algo_core_selftest COMMAND algo_core_selftest)
endif()

ament_package()
