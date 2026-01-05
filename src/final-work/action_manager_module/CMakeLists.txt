cmake_minimum_required(VERSION 3.8)
project(action_manager_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(action_manager_module_interfaces REQUIRED)

find_package(sensing_module REQUIRED)
find_package(planning_module REQUIRED)

find_package(robotiq_85_gripper_server REQUIRED)

add_executable(action_manager_server src/action_manager_server.cpp)

ament_target_dependencies(action_manager_server
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  action_manager_module_interfaces
  sensing_module
  planning_module
  robotiq_85_gripper_server
)

install(TARGETS action_manager_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
