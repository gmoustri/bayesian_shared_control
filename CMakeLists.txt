cmake_minimum_required(VERSION 3.20)
project(bayesian_shared_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Dependencies you actually use
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# DWAL msgs (consumer)
find_package(dwal_planner REQUIRED)

# ---------------------------
# Shared Controller (object library)
# ---------------------------
add_library(shared_controller_obj OBJECT
  src/shared_controller.cpp
  src/helper_fcns.cpp
)

target_include_directories(shared_controller_obj PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Modern linking (Kilted+)
target_link_libraries(shared_controller_obj PUBLIC
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${nav2_costmap_2d_TARGETS}
  ${dwal_planner_TARGETS}
  Boost::system
)

# ---------------------------
# Node executable
# ---------------------------
add_executable(shared_control_node
  src/shared_control_node.cpp
  $<TARGET_OBJECTS:shared_controller_obj>
)

# The node needs your include/ too (for #include "shared_controller/shared_controller.h")
target_include_directories(shared_control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link node to deps too (because it compiles its own TU and includes headers)
target_link_libraries(shared_control_node PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${nav2_costmap_2d_TARGETS}
  ${dwal_planner_TARGETS}
  Boost::system
)

# ---------------------------
# Install
# ---------------------------
install(TARGETS
  shared_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/teleop_keyboard.py
  scripts/remote_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------
# Export (only needed if other packages link to your headers/libs)
# ---------------------------
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs trajectory_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs nav2_costmap_2d
  dwal_planner
)

ament_package()
