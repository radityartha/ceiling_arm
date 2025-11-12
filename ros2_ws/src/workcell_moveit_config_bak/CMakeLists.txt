cmake_minimum_required(VERSION 3.8)
project(workcell_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
