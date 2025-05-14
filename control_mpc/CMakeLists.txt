# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.8)

# Define the project name and language
project(control_mpc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -g -march=native) Debug info
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(sdf_optimized_msgs REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include header directories
include_directories(include
  ${YAML_CPP_INCLUDE_DIRS}      # YAML-CPP headers
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIRS}
)

# Add executable
add_executable(control_mpc 
  src/main.cpp
  src/Control_MPC.cpp
  src/MPC.cpp
  src/TFSubscriberNode.cpp
  src/Matrix.cpp
)

# Link libraries to target
ament_target_dependencies(control_mpc
  rclcpp                     # ROS2 C++ client library
  std_msgs                   # Standard messages
  sensor_msgs                # Sensor messages
  nav_msgs                   # Rviz Goal messages
  yaml-cpp                   # YAML-CPP library
  sdf_optimized_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Link libraries
target_link_libraries(control_mpc
  yaml-cpp
  OsqpEigen::OsqpEigen
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

# Install executable to specified directory
install(TARGETS control_mpc
  DESTINATION lib/${PROJECT_NAME}
)

# Declare ament package (must be at the end)
ament_package()
