cmake_minimum_required(VERSION 3.12)
project(topographic_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Core library
add_library(topographic_planner_core STATIC
  src/pathfinder.cpp
  src/smooth.cpp
)
target_include_directories(topographic_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(topographic_planner_core grid_map_core)

# ROS node
add_executable(topographic_planner_node
  src/action_server.cpp
  src/main.cpp
)
target_link_libraries(topographic_planner_node topographic_planner_core)
ament_target_dependencies(topographic_planner_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  nav2_msgs
  nav2_util
  grid_map_ros
  grid_map_core
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  topographic_planner_core
  topographic_planner_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(topographic_planner_core)
ament_package()
