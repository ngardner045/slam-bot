cmake_minimum_required(VERSION 3.8)
project(slam_bot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY
  urdf
  launch
  config
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
