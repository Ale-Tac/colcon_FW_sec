cmake_minimum_required(VERSION 3.8)
project(action_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(sensing_module REQUIRED)
find_package(planning_module REQUIRED)
find_package(aruco_broadcaster REQUIRED)
find_package(chesslab_setup2_interfaces REQUIRED)
find_package(robotiq_85_gripper_server REQUIRED)
find_package(kinenikros2 REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ChessAction.action"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

# Add executable
add_executable(action_manager_node src/action_manager_node.cpp)

ament_target_dependencies(action_manager_node
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  sensing_module
  planning_module
  aruco_broadcaster
  chesslab_setup2_interfaces
  robotiq_85_gripper_server
  kinenikros2
)

rosidl_target_interfaces(action_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS action_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
