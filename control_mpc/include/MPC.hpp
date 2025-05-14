#ifndef MPC_HPP
#define MPC_HPP

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "Eigen/Dense"
#include "OsqpEigen/OsqpEigen.h"
#include "../../route_planning/include/utils/minco.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Control_MPC.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "TFSubscriberNode.hpp"

typedef struct 
{
    double L;
    double W;     
    double wheel_y[2];
    double wheel_x[5];

    double V_l[5];
    double V_r[5];
    double gamma_l[5];
    double gamma_r[5];

    double Alpha;
    double P;
    double beta[5];
    double k_alpha;
    double k_p;
    double T;
    double Velocity;


}car_info_;


class MpcNode : public rclcpp::Node {
public:
    MpcNode(std::shared_ptr<ControlNode> control_node, std::shared_ptr<TFSubscriberNode> tf_subscriber_node);
private:
    void TimerCallback();
    void findCurrentSegmentAndLocalTime(double t, int & segment, double & local_t, int pieceN, Eigen::VectorXd times);
    Eigen::Vector3d findclosetpoint(int segment, int pointsN, Eigen::VectorXd times, Eigen::MatrixX3d b);
    Eigen::Vector3d findClosestPointOnSegment(int segment, Eigen::VectorXd times, Eigen::MatrixX3d b);
    void calculateWheelSettings(double vx, double vy, double& angle, double& speed);
    void Control(Eigen::Vector3d Control);
    Eigen::MatrixXd kroneckerProduct(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B); // 计算矩阵的克罗内克积
    Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& matrix, int power);
    void checkMatrix(Eigen::MatrixXd& eigenH);
    void isSymmetric(Eigen::MatrixXd& matrix);

    //控制话题定义
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_string_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_speed_publisher_;

    std::shared_ptr<ControlNode> control_node_;  // control_node共享指针
    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;  // tf_subscriber_node共享指针
    // 车辆控制信息发布
    std_msgs::msg::Float64MultiArray array_msg;     //转向控制
    std_msgs::msg::Float64MultiArray speed_msg;     //转向控制

    car_info_ car_info;
    std::string config_yaml_path;

    //MPC参数
    int Np = 10;                   //预测步长
    int Nc = 5;                    //控制步长
    double T = 0.05;             //控制周期0.05s
    double speed_ratio;
    Matrix Odom_Car_Matrix;     // 世界中心 -> 车辆 变换矩阵
    Matrix Car_Odom_Matrix;
    Eigen::Vector3d u_last;
    rclcpp::TimerBase::SharedPtr timer_;

    minco::MINCO_S3NU minco;
    Eigen::MatrixXd Car_Trajectory_Rotation_Translation_Matrix;
    Eigen::MatrixXd Car_Trajectory_Rotation_Matrix;
};

#endif