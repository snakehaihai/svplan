Swept Volume-Aware Trajectory Planning and MPC Tracking for Multi-Axle AMRs
==============================================================================

This repository contains the official implementation of the paper:
"Swept Volume-Aware Trajectory Planning and MPC Tracking for Multi-Axle Swerve-Drive AMRs"
arXiv:2412.16875v2

Tested Environment
------------------

- Ubuntu 22.04
- ROS 2 Humble
- CUDA 12.2
- Gazebo + RViz

Dependencies
------------

Install required ROS 2 packages:

sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager  
sudo apt install ros-humble-xacro  
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control wget

Install Eigen 3.4.0:

wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz  
tar -zxvf eigen-3.4.0.tar.gz && rm eigen-3.4.0.tar.gz  
cd eigen-3.4.0 && mkdir build && cd build  
cmake .. && make -j$(nproc) && sudo make install  
cd ../..

Install OSQP v1.0.0:

git clone --branch v1.0.0 https://github.com/osqp/osqp.git  
cd osqp && mkdir build && cd build  
cmake .. -DCMAKE_BUILD_TYPE=Release  
make -j$(nproc) && sudo make install  
cd ../..

Install OSQP-Eigen v0.10.0:

git clone --branch v0.10.0 https://github.com/robotology/osqp-eigen.git  
cd osqp-eigen && mkdir build && cd build  
cmake .. && make -j$(nproc) && sudo make install  
cd ../..

Install LBFGSpp v0.4.0:

git clone --branch v0.4.0 https://github.com/yixuan/LBFGSpp.git  
cd LBFGSpp && mkdir build && cd build  
cmake .. -DCMAKE_BUILD_TYPE=Release  
make -j$(nproc) && sudo make install  
cd ../..

Install PCL v1.14.0:

sudo apt-get install libflann-dev libusb-1.0-0-dev libopenni-dev libopenni2-dev libboost-all-dev libeigen3-dev clang-format libqhull-dev libpcap-dev freeglut3-dev libpng-dev libglew-dev  
git clone --branch v1.14.0 https://github.com/PointCloudLibrary/pcl.git  
cd pcl && mkdir build && cd build  
cmake .. -DCMAKE_BUILD_TYPE=Release  
make -j$(nproc) && sudo make install  
cd ../..

Add to .bashrc:

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

Build Instructions
------------------

1. Create workspace and clone code:

mkdir -p ~/multi-axle-all-wheel-steering-vehicles_ws/src  
cd ~/multi-axle-all-wheel-steering-vehicles_ws/src  
git clone https://github.com/ccwss-maker/svplan.git

2. Copy Gazebo models:

cp -r svplan/multi_axle_vehicle_model/worlds/models ~/.gazebo/

3. Manually modify the following files to set correct config_yaml_path:

- route_planning/src/A_Star_Node.cpp  
- route_planning/src/TrajectoryOpimizationNode.cpp  
- route_planning/src/SDFOpimizationNode.cpp  
- control_mpc/src/Control_MPC.cpp  
- control_mpc/src/MPC.cpp

4. Build the workspace:

cd ~/multi-axle-all-wheel-steering-vehicles_ws  
colcon build

5. Source setup script:

source ~/multi-axle-all-wheel-steering-vehicles_ws/install/setup.bash

Launch & Run
------------

ros2 launch multi_axle_vehicle_model car.launch.py  
ros2 run route_planning route_planning  
ros2 run control_mpc control_mpc

Citation
--------

If you find this work helpful, please cite:

@article{hu2024swept,  
  title={Swept Volume-Aware Trajectory Planning and MPC Tracking for Multi-Axle Swerve-Drive AMRs},  
  author={Hu, Tianxin and Yuan, Shenghai and Bai, Ruofei and Xu, Xinhang and Liao, Yuwen and Liu, Fen and Xie, Lihua},  
  journal={arXiv preprint arXiv:2412.16875},  
  year={2024}  
}
