#ifndef SDF_CUH
#define SDF_CUH

#include "yaml-cpp/yaml.h"
#include "Eigen/Dense"

typedef struct{
    int pieceN;
    Eigen::MatrixX3d b;      
    Eigen::VectorXd times;
    Eigen::Matrix3Xd points;   
}Optimized_Trajectory_;

typedef struct{
    int pieceN;
    double *b;      
    double *times;
}GPU_Initial_Optimized_Trajectory_;

typedef struct{
    double x;
    double y;      
    double SDF;
}SDF_Map_;

void GPUProcessConfig(YAML::Node config);
void GPUProcessSDF(Optimized_Trajectory_ init_traj, std::vector<SDF_Map_>& SDF_Map, double& area, bool test);
void GPUProcessGradSDF(Optimized_Trajectory_ traj, Eigen::Matrix3Xd Obstacle_Points, Eigen::MatrixX3d & GradByPositions, Eigen::VectorXd & GradByTimes, double & cost);
void GPUProcessGradYaw(Optimized_Trajectory_ traj, Eigen::MatrixX3d & GradByPoints_Yaw, Eigen::VectorXd & GradByTimes_Yaw, double & cost);
#endif