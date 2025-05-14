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
- Eigen 3.4.0
- OSQP v1.0.0
- OSQP-Eigen v0.10.0
- LBFGSpp v0.4.0
- PCL v1.14.0

Dependencies
------------

Install required ROS 2 packages:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-controller-manager ros-humble-xacro ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control wget
```

To install third-party libraries manually (skip if already installed):

Install Eigen 3.4.0:
```bash
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -zxvf eigen-3.4.0.tar.gz && rm eigen-3.4.0.tar.gz
cd eigen-3.4.0 && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
cd ../..
```

Install OSQP v1.0.0:
```bash
git clone --branch v1.0.0 https://github.com/osqp/osqp.git
cd osqp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install
cd ../..
```

Install OSQP-Eigen v0.10.0:
```bash
git clone --branch v0.10.0 https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
cd ../..
```

Install LBFGSpp v0.4.0:
```bash
git clone --branch v0.4.0 https://github.com/yixuan/LBFGSpp.git
cd LBFGSpp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install
cd ../..
```

Install PCL v1.14.0:
```bash
sudo apt-get install libflann-dev libusb-1.0-0-dev libopenni-dev libopenni2-dev libboost-all-dev libeigen3-dev clang-format libqhull-dev libpcap-dev freeglut3-dev libpng-dev libglew-dev
git clone --branch v1.14.0 https://github.com/PointCloudLibrary/pcl.git
cd pcl && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) && sudo make install
cd ../..
```

Add to .bashrc:
```bash
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
```

Build Instructions
------------------

1. Create workspace and clone code:
```bash
mkdir -p ~/multi-axle-all-wheel-steering-vehicles_ws/src
cd ~/multi-axle-all-wheel-steering-vehicles_ws/src
git clone https://github.com/ccwss-maker/svplan.git
```

2. Copy Gazebo models:
```bash
cp -r svplan/multi_axle_vehicle_model/worlds/models ~/.gazebo/
```

3. Manually update the `config_yaml_path` in the following 5 source files:

```cpp
// For the route planning module
src/svplan/route_planning/src/A_Star_Node.cpp
src/svplan/route_planning/src/TrajectoryOpimizationNode.cpp
src/svplan/route_planning/src/SDFOpimizationNode.cpp

// Replace with your actual path:
config_yaml_path = "/absolute/path/to/your/multi-axle-all-wheel-steering-vehicles_ws/src/route_planning/config/route_planning_config.yaml";

// For the MPC control module
src/svplan/control_mpc/src/Control_MPC.cpp
src/svplan/control_mpc/src/MPC.cpp

// Replace with your actual path:
config_yaml_path = "/absolute/path/to/your/multi-axle-all-wheel-steering-vehicles_ws/src/control_mpc/config/size.yaml";
```

4. Build the workspace:
```bash
cd ~/multi-axle-all-wheel-steering-vehicles_ws
colcon build
```

5. Source setup script:
```bash
echo "source ~/multi-axle-all-wheel-steering-vehicles_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Launch & Run
------------

```bash
ros2 launch multi_axle_vehicle_model car.launch.py
ros2 run route_planning route_planning
ros2 run control_mpc control_mpc
```

Docker Image
------------

A pre-configured Docker image is available:
```bash
docker push ccwssplus/svplan:latest
```

Citation
--------

If you find this work helpful, please cite:
```bibtex
@article{hu2024swept,
  title={Swept Volume-Aware Trajectory Planning and MPC Tracking for Multi-Axle Swerve-Drive AMRs},
  author={Hu, Tianxin and Yuan, Shenghai and Bai, Ruofei and Xu, Xinhang and Liao, Yuwen and Liu, Fen and Xie, Lihua},
  journal={arXiv preprint arXiv:2412.16875},
  year={2024}
}
```
