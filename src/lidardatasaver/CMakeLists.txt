cmake_minimum_required(VERSION 3.8)
project(lidardatasaver)

cmake_policy(SET CMP0074 NEW)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add executable
add_executable(${PROJECT_NAME}_node src/lidardatasaver.cpp)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)

target_link_libraries(${PROJECT_NAME}_node
  yaml-cpp
  ${PCL_LIBRARIES}
)

# Install executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Register package
ament_package()
