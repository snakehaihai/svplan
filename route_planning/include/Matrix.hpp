#ifndef MATRIX_HPP
#define MATRIX_HPP

#include "rclcpp/rclcpp.hpp"
#include "Eigen/Dense"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class Matrix {
public:
    Matrix() = default;
    void Rotation_Set(double roll, double pitch, double yaw);
    void Translation_Set(double x, double y, double z);
    void Rotation_Translation_Set();
    Eigen::Matrix3d Rotation_Read();
    Eigen::Vector3d Translation_Read();
    Eigen::MatrixXd Rotation_Translation_Read();
    geometry_msgs::msg::Quaternion Quaternion_Read();
    void setFromTF(geometry_msgs::msg::TransformStamped transformStamped);

private:
    geometry_msgs::msg::Quaternion Quaternion;  // 四元数
    Eigen::Matrix3d Rotation;                   // 旋转矩阵3*3
    Eigen::Vector3d Translation;                // 平移矩阵3*1
    Eigen::MatrixXd Rotation_Translation;       // 旋转平移矩阵3*4
};

#endif // MATRIX_HPP
