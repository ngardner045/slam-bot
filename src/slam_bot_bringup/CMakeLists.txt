cmake_minimum_required(VERSION 3.8)
project(slam_bot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(launch_ros REQUIRED)
find_package(launch REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
install(PROGRAMS
  scripts/robot_bringup.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
