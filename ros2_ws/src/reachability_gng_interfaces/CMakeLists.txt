cmake_minimum_required(VERSION 3.8)
project(reachability_gng_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# SampleGrasps returns PoseArray / Vector3 / Point, so the generated code needs
# geometry_msgs' definitions available at build time.
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SampleGrasps.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
