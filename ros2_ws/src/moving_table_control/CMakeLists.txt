cmake_minimum_required(VERSION 3.8)
project(moving_table_control)

# Find necessary dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#
# --- This is the part that builds your .srv file ---
#
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MovingTable.srv"
)

#
# --- This part runs your setup.py ---
#
find_package(ament_python REQUIRED)
ament_python_install_package()

#
# --- Install launch files and URDFs ---
#
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
