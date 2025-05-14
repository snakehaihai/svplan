#ifndef CONTROL_MPC_HPP
#define CONTROL_MPC_HPP

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sdf_optimized_msgs/msg/sdf_optimized_trajectory.hpp"
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include "../../route_planning/include/utils/minco.hpp"
#include "Matrix.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"



class ControlNode : public rclcpp::Node {
public:
    ControlNode();
    
    typedef struct{
        int pieceN;
        double T_total;
        Eigen::MatrixX3d b;      
        Eigen::VectorXd times;
        Eigen::Matrix3Xd init_points;   
        Eigen::MatrixXd Odom_Car_Rotation_Translation_Matrix;
    }Trajectory_;
    Trajectory_ Trajectory;

    Eigen::Matrix3Xd control_points;
private:
    void Callback(const sdf_optimized_msgs::msg::SDFOptimizedTrajectory::ConstSharedPtr & traj);
    //话题订阅
    rclcpp::Subscription<sdf_optimized_msgs::msg::SDFOptimizedTrajectory>::SharedPtr Traj_Subscriber_;

    std::string config_yaml_path;
    minco::MINCO_S3NU minco;
};

#endif