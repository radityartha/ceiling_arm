cmake_minimum_required(VERSION 3.8)
project(moving_table_pkg)

# Find necessary build tools and dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# This includes the file that defines ament_python_setup()
# This is the correct file to include.
include(ament_python_package)

# Define the service files to be built
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MovingTable.srv"
)

# Call the setup.py for Python-specific tasks
# (finding packages, entry points, etc.)
ament_python_setup()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install robot models
install(DIRECTORY
  robots
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

