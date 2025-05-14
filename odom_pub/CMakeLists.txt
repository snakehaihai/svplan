cmake_minimum_required(VERSION 3.5)
project(odom_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include ${gazebo_dev_INCLUDE_DIRS} ${gazebo_ros_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${tf2_ros_INCLUDE_DIRS})

# Create the plugin
add_library(${PROJECT_NAME} SHARED
  src/OdomPlugin.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
)

# Install the plugin
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files and other resources
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
