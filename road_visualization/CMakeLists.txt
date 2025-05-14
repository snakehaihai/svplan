cmake_minimum_required(VERSION 3.10)

project(road_visualization)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(road_publisher_node src/road_publisher_node.cpp)

ament_target_dependencies(road_publisher_node
  rclcpp
  visualization_msgs
)

install(TARGETS
  road_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
