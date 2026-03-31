cmake_minimum_required(VERSION 3.8)
project(open_manipulator_x_playground)

###############################################################################
# Compile options
###############################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# Dependencies
###############################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

###############################################################################
# Include directory (GLOBAL)
###############################################################################
include_directories(
  include
)

###############################################################################
# hello_moveit node
###############################################################################
add_executable(hello_moveit
  src/hello_moveit.cpp
)

ament_target_dependencies(
  hello_moveit
  rclcpp
  moveit_ros_planning_interface
)

###############################################################################
# main node
###############################################################################
add_executable(main
  src/main.cpp
)

ament_target_dependencies(
  main
  rclcpp
  moveit_ros_planning_interface
)

###############################################################################
# Install
###############################################################################
install(TARGETS
  hello_moveit
  main
  DESTINATION lib/${PROJECT_NAME}
)

###############################################################################
# Ament
###############################################################################
ament_package()