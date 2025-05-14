# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.8)

# 定义项目名称及使用的语言
project(control_mpc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -g -march=native) 调试信息
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 查找需要的包
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

# 包含头文件目录
include_directories(include
  ${YAML_CPP_INCLUDE_DIRS}      # YAML-CPP头文件目录
  ${EIGEN3_INCLUDE_DIR}
  ${OsqpEigen_INCLUDE_DIRS}
)
# 添加可执行文件
add_executable(control_mpc 
  src/main.cpp
  src/Control_MPC.cpp
  src/MPC.cpp
  src/TFSubscriberNode.cpp
  src/Matrix.cpp
)

# 链接库文件
ament_target_dependencies(control_mpc
  rclcpp                     # ROS2 C++客户端库
  std_msgs                   # 标准消息
  sensor_msgs                # 传感器消息
  nav_msgs              # Rviz Goal消息
  yaml-cpp                   # YAML-CPP库
  sdf_optimized_msgs
  tf2_ros
  tf2_geometry_msgs
)

# 链接库
target_link_libraries(control_mpc
  yaml-cpp
  OsqpEigen::OsqpEigen
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

# 安装可执行文件到指定目录
install(TARGETS control_mpc
  DESTINATION lib/${PROJECT_NAME}
)

# 定义ament包，这个指令必须在最后
ament_package()