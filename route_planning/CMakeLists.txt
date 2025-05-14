# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.8)

# 定义项目名称及使用的语言
project(route_planning LANGUAGES CXX CUDA Fortran)

# 设置CUDA相关变量
set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 89 90)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -g -march=native) #调试信息
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()


# 查找CUDA包和设置相关变量
find_package(CUDA REQUIRED)
set(CUDA_LIB_DIR "/usr/local/cuda/lib64")

# 查找需要的包
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

# 包含头文件目录
include_directories(include
  ${CUDA_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}      # YAML-CPP头文件目录
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

# 添加可执行文件
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

# 链接库文件
ament_target_dependencies(route_planning
  rclcpp                      # ROS2 C++客户端库
  std_msgs                    # 标准消息
  sensor_msgs                 # 传感器消息
  geometry_msgs               # Rviz Goal消息
  astar_msgs
  initial_optimized_msgs
  sdf_optimized_msgs
  visualization_msgs
  message_filters             # 消息过滤器
  yaml-cpp                    # YAML-CPP库
  nav_msgs
  pcl_conversions
  PCL
  tf2_ros
  tf2_geometry_msgs
)
# 链接库
target_link_libraries(route_planning
  yaml-cpp
  cudart
  ${PCL_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  gfortran
)

# 安装可执行文件到指定目录
install(TARGETS route_planning
  DESTINATION lib/${PROJECT_NAME}
)

# 定义ament包，这个指令必须在最后
ament_package()