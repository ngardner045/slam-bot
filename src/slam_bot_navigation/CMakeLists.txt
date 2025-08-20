cmake_minimum_required(VERSION 3.8)
project(slam_bot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and configuration
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
