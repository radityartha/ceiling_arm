cmake_minimum_required(VERSION 3.8)
project(workcell_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(kortex_description REQUIRED) # For including kinova macros

# --- Corrected Install Section ---
# Install each directory individually.

# Install the URDF directory (which we know exists)
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Optionally install other directories *if they exist*
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/rviz")
  install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/meshes")
  install(
    DIRECTORY meshes
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(PROGRAMS
  scripts/lidar_filter.py
  scripts/run_take_bag.py
  scripts/run_take_bottle.py
  scripts/run_close_curtain.py
  scripts/run_open_curtain.py
  scripts/run_go_home.py
  DESTINATION lib/${PROJECT_NAME}
)
# --- End Corrected Install Section ---

ament_package()