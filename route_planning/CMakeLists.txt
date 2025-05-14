# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.8)

# Define the project name and used languages
project(route_planning LANGUAGES CXX CUDA Fortran)

# Set CUDA-related variables
set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 89 90)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

# Set compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -g -march=native) # Debug info
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# Find CUDA package and set related variables
find_package(CUDA REQUIRED)
set(CUDA_LIB_DIR "/usr/local/cuda/lib64")

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(astar_msgs REQUIRED)
find_package(initial_optimized_msgs REQUIRED)
find_package(sdf_optimized_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation visualization)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include header directories
include_directories(include
  ${CUDA_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}      # YAML-CPP headers
  ${EIGEN3_INCLUDE_DIR}
  ${GSL_INCLUDE_DIRS}
  ${a_star_msgs_INCLUDE_DIRS}
)

set(FORTRAN_SOURCES
    include/utils/lmbm_call.f
    include/utils/lmbm_main.f
    include/utils/lmbm_math.f
    include/utils/lmbm_sub.f
)

# Add executable
add_executable(route_planning 
  src/A_Star_Node.cpp
  src/TFSubscriberNode.cpp
  src/Matrix.cpp
  src/TrajectoryOpimizationNode.cpp
  src/SDFOpimizationNode.cpp
  src/main.cpp
  src/SDF.cu
  include/utils/lmbm.cpp
  ${FORTRAN_SOURCES}
)

# Link libraries to target
ament_target_dependencies(route_planning
  rclcpp                      # ROS2 C++ client library
  std_msgs                    # Standard messages
  sensor_msgs                 # Sensor messages
  geometry_msgs               # Rviz Goal messages
  astar_msgs
  initial_optimized_msgs
  sdf_optimized_msgs
  visualization_msgs
  message_filters             # Message filters
  yaml-cpp                    # YAML-CPP library
  nav_msgs
  pcl_conversions
  PCL
  tf2_ros
  tf2_geometry_msgs
)

# Link libraries
target_link_libraries(route_planning
  yaml-cpp
  cudart
  ${PCL_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  gfortran
)

# Install executable to the specified directory
install(TARGETS route_planning
  DESTINATION lib/${PROJECT_NAME}
)

# Declare ament package (must be at the end)
ament_package()
