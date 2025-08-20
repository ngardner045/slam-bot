cmake_minimum_required(VERSION 3.8)
project(slam_bot_workspace)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Include directories
include_directories(include)

# Add subdirectories for each package
add_subdirectory(slam_bot_bringup)
add_subdirectory(slam_bot_control)
add_subdirectory(slam_bot_description)
add_subdirectory(slam_bot_navigation)

ament_package()
