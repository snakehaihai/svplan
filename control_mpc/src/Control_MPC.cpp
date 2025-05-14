#include "Control_MPC.hpp"
#include "math.h"

ControlNode::ControlNode() : Node("control_node") {
    // 初始化订阅者
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
    // std::cout<<Trajectory.init_points<<std::endl;
    // Eigen::Matrix3d initState = Eigen::Matrix3d::Zero();
    // Eigen::Matrix3d finalState = Eigen::Matrix3d::Zero();
    // initState.col(0) = Trajectory.init_points.col(0);
    // finalState.col(0) = Trajectory.init_points.col(Trajectory.pieceN);
    // minco.setConditions(initState, finalState, Trajectory.pieceN);
    // Eigen::Matrix3Xd points = Trajectory.init_points.block(0, 1, 3, Trajectory.pieceN - 1);
    // minco.setParameters(points, Trajectory.times);
    // Trajectory.b = minco.b;

    // YAML::Node config = YAML::LoadFile(config_yaml_path);
    // double T = config["T"].as<double>();

    // int step = (int)(Trajectory.T_total / T);
    // std::cout<<step<<std::endl;
    // control_points.resize(3, step);
    // for(int i=0; i<step; i++)
    // {
    //     int current_segment;
    //     double local_t;
    //     double t = i * T;
    //     findCurrentSegmentAndLocalTime(t, current_segment, local_t);
    //     double t0 = 1;
    //     double t1 = local_t;
    //     double t2 = t1 * t1;
    //     double t3 = t1 * t2;
    //     double t4 = t1 * t3;
    //     double t5 = t1 * t4;
    //     Eigen::VectorXd T_Position(6);
    //     T_Position << t0, t1, t2, t3, t4, t5;
    //     Eigen::MatrixX3d group = Trajectory.b.block(current_segment * 6, 0, 6, 3).transpose();
    //     control_points.block(0, i, 3, 1) = group * T_Position;
    // }
}