#include "Control_MPC.hpp"
#include "math.h"

ControlNode::ControlNode() : Node("control_node") {
    Traj_Subscriber_ = this->create_subscription<sdf_optimized_msgs::msg::SDFOptimizedTrajectory>("/SDF_opimization_trajectory", 10, std::bind(&ControlNode::Callback, this, std::placeholders::_1));
    config_yaml_path = "/root/PersonalData/Program/multi-axle-all-wheel-steering-vehicles_ws/src/control_mpc/config/size.yaml";
}

void ControlNode::Callback(const sdf_optimized_msgs::msg::SDFOptimizedTrajectory::ConstSharedPtr & traj)
{
    Trajectory.Odom_Car_Rotation_Translation_Matrix = Eigen::MatrixXd::Zero(4, 4);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Trajectory.Odom_Car_Rotation_Translation_Matrix(i, j) = traj->rotation_translation_matrix.data[i * 4 + j];
        }
    }
    
    Trajectory.pieceN = traj->times.size();
    Trajectory.times.resize(traj->times.size());
    Trajectory.init_points.resize(3, traj->position.size());

    for(int i=0; i<traj->times.size(); i++)
    {
        Trajectory.times(i) = traj->times[i];
    }
    for(int i=0; i<traj->position.size(); i++)
    {
        Trajectory.init_points.col(i) << traj->position[i].x,
                                         traj->position[i].y,
                                         traj->position[i].z;
    }
    Trajectory.T_total = Trajectory.times.sum();

    std::cout<<"Trajectory.init_points.cols() = "<<Trajectory.init_points.cols()<<std::endl;
    std::cout<<"Trajectory.times.size() = "<<Trajectory.times.size()<<std::endl;
}