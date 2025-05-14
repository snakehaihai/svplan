#ifndef ROUTE_PLANNING_HPP
#define ROUTE_PLANNING_HPP
#include "TFSubscriberNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "queue"
#include "yaml-cpp/yaml.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "astar_msgs/msg/a_star_path_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "Eigen/Dense"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

struct AStarNode {
    int x, y;
    float g_cost, h_cost;
    int parent_x, parent_y;

    AStarNode(int x, int y, float g, float h, int px, int py) 
        : x(x), y(y), g_cost(g), h_cost(h), parent_x(px), parent_y(py) {}

    float f_cost() const {
        return g_cost + h_cost;
    }

    bool operator>(const AStarNode& other) const {
        return f_cost() > other.f_cost();
    }
};

typedef struct{
    Eigen::Vector2d position;
    double yaw; 
}A_Star_Path_;

class A_Star_Node : public rclcpp::Node {
public:
    A_Star_Node(std::shared_ptr<TFSubscriberNode> tf_subscriber_node);

private:
    void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & point_cloud_msg, const geometry_msgs::msg::PoseStamped::ConstSharedPtr & goal_msg);
    std::vector<A_Star_Path_> AStarSearch(const std::vector<std::vector<int>>& grid_map, const std::pair<int, int>& start, const std::pair<int, int>& goal, double grid_resolution_meters);
    bool isPointOnRoad(const std::array<double, 2>& point, const std::array<double, 2>& start, const std::array<double, 2>& end, double width);
    // 定义同步策略别名
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, geometry_msgs::msg::PoseStamped> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> MySynchronizer;
    std::shared_ptr<MySynchronizer> sync_; // 现在是类级别的别名

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> point_cloud_msg_;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> goal_msg_;

    //控制话题定义
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Filter_Publisher;   //发布滤波点云话题
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Road_Side_Publisher_;       //发布道路边界障碍点云
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr A_Star_Path_Rviz_Publisher_;       //发布A*路径
    rclcpp::Publisher<astar_msgs::msg::AStarPathArray>::SharedPtr A_Star_Path_Publisher_;       //发布A*路径
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr A_Star_Map_Publisher_;       //发布A*地图

    std::shared_ptr<TFSubscriberNode> tf_subscriber_node_;  // tf_subscriber_node共享指针
    std::vector<A_Star_Path_> A_Star_Path;

    std::string config_yaml_path;
    YAML::Node config;
    Matrix Lidar_Car_Matrix;    // 激光雷达 -> 车辆 变换矩阵
    Matrix Odom_Car_Matrix;     // 世界中心 -> 车辆 变换矩阵
    Matrix Car_Odom_Matrix;      // 车辆 -> 世界中心 变换矩阵


    std::vector<std::tuple<std::array<double, 2>, std::array<double, 2>>> 
    detectRoadSegments(double x_min, double x_max, double y_min, double y_max, 
                    const std::vector<std::tuple<std::array<double, 2>, std::array<double, 2>>>& roads_side) {
        std::vector<std::tuple<std::array<double, 2>, std::array<double, 2>>> result;

        for (const auto& road : roads_side) {
            auto [start, end] = road;
            double x1 = start[0], y1 = start[1];
            double x2 = end[0], y2 = end[1];

            // 检查线段是否与矩形相交
            double t_min = 0, t_max = 1;
            
            // 检查 x 方向
            if (std::abs(x2 - x1) > 1e-6) {
                double tx1 = (x_min - x1) / (x2 - x1);
                double tx2 = (x_max - x1) / (x2 - x1);
                t_min = std::max(t_min, std::min(tx1, tx2));
                t_max = std::min(t_max, std::max(tx1, tx2));
            } else if (x1 < x_min || x1 > x_max) {
                continue;
            }

            // 检查 y 方向
            if (std::abs(y2 - y1) > 1e-6) {
                double ty1 = (y_min - y1) / (y2 - y1);
                double ty2 = (y_max - y1) / (y2 - y1);
                t_min = std::max(t_min, std::min(ty1, ty2));
                t_max = std::min(t_max, std::max(ty1, ty2));
            } else if (y1 < y_min || y1 > y_max) {
                continue;
            }

            if (t_max >= t_min) {
                double new_x1 = x1 + t_min * (x2 - x1);
                double new_y1 = y1 + t_min * (y2 - y1);
                double new_x2 = x1 + t_max * (x2 - x1);
                double new_y2 = y1 + t_max * (y2 - y1);

                result.push_back({
                    {std::max(x_min, std::min(x_max, new_x1)), 
                    std::max(y_min, std::min(y_max, new_y1))},
                    {std::max(x_min, std::min(x_max, new_x2)), 
                    std::max(y_min, std::min(y_max, new_y2))}
                });
            }
        }

        return result;
    }


    std::vector<std::tuple<std::array<double, 2>, std::array<double, 2>, double>> roads = {
            {{-45, -103.7}, {-45, 104.2}, 7.4},
            {{-15, -103.7}, {-15, 104.2}, 7.4},
            {{45, -103.7}, {45, 104.2}, 7.4},
            {{110, -103.7}, {110, 104.2}, 7.4},
            {{120, -103.7}, {120, 104.2}, 7.4},

            {{-41.3, -100}, {116.3, -100}, 8.4},
            {{-41.3, -45}, {116.3, -45}, 8.4},
            {{-41.3, 0}, {116.3, 0}, 8.4},
            {{-41.3, 45}, {116.3, 45}, 8.4},
            {{-41.3, 100}, {116.3, 100}, 8.4}
    };

    std::vector<std::tuple<std::array<double, 2>, std::array<double, 2>>> roads_side = {
        {{-48.7, -103.7}, {-48.7, 104.2}},

        {{-41.3, -95.8}, {-41.3, -49.2}},
        {{-41.3, -40.8}, {-41.3, -4.2}},
        {{-41.3, 4.2}, {-41.3, 40.8}},
        {{-41.3, 49.2}, {-41.3, 95.8}},

        {{-18.7, -95.8}, {-18.7, -49.2}},
        {{-18.7, -40.8}, {-18.7, -4.2}},
        {{-18.7, 4.2}, {-18.7, 40.8}},
        {{-18.7, 49.2}, {-18.7, 95.8}},

        {{-11.3, -95.8}, {-11.3, -49.2}},
        {{-11.3, -40.8}, {-11.3, -4.2}},
        {{-11.3, 4.2}, {-11.3, 40.8}},
        {{-11.3, 49.2}, {-11.3, 95.8}},

        {{41.3, -95.8}, {41.3, -49.2}},
        {{41.3, -40.8}, {41.3, -4.2}},
        {{41.3, 4.2}, {41.3, 40.8}},
        {{41.3, 49.2}, {41.3, 95.8}},

        {{48.7, -95.8}, {48.7, -49.2}},
        {{48.7, -40.8}, {48.7, -4.2}},
        {{48.7, 4.2}, {48.7, 40.8}},
        {{48.7, 49.2}, {48.7, 95.8}},

        {{106.3, -95.8}, {106.3, -49.2}},
        {{106.3, -40.8}, {106.3, -4.2}},
        {{106.3, 4.2}, {106.3, 40.8}},
        {{106.3, 49.2}, {106.3, 95.8}},

        {{113.7, -95.8}, {113.7, -49.2}},
        {{113.7, -40.8}, {113.7, -4.2}},
        {{113.7, 4.2}, {113.7, 40.8}},
        {{113.7, 49.2}, {113.7, 95.8}},

        {{116.3, -95.8}, {116.3, -49.2}},
        {{116.3, -40.8}, {116.3, -4.2}},
        {{116.3, 4.2}, {116.3, 40.8}},
        {{116.3, 49.2}, {116.3, 95.8}},

        {{123.7, -103.7}, {123.7, 104.2}},

        {{-48.7, -104.2}, {123.7, -104.2}},

        {{-41.3, -95.8}, {-18.7, -95.8}},
        {{-11.3, -95.8}, {41.3, -95.8}},
        {{48.7, -95.8}, {106.3, -95.8}},
        {{113.7, -95.8}, {116.3, -95.8}},

        {{-41.3, -49.2}, {-18.7, -49.2}},
        {{-11.3, -49.2}, {41.3, -49.2}},
        {{48.7, -49.2}, {106.3, -49.2}},
        {{113.7, -49.2}, {116.3, -49.2}},

        {{-41.3, -40.8}, {-18.7, -40.8}},
        {{-11.3, -40.8}, {41.3, -40.8}},
        {{48.7, -40.8}, {106.3, -40.8}},
        {{113.7, -40.8}, {116.3, -40.8}},

        {{-41.3, -4.2}, {-18.7, -4.2}},
        {{-11.3, -4.2}, {41.3, -4.2}},
        {{48.7, -4.2}, {106.3, -4.2}},
        {{113.7, -4.2}, {116.3, -4.2}},

        {{-41.3, 4.2}, {-18.7, 4.2}},
        {{-11.3, 4.2}, {41.3, 4.2}},
        {{48.7, 4.2}, {106.3, 4.2}},
        {{113.7, 4.2}, {116.3, 4.2}},

        {{-41.3, 40.8}, {-18.7, 40.8}},
        {{-11.3, 40.8}, {41.3, 40.8}},
        {{48.7, 40.8}, {106.3, 40.8}},
        {{113.7, 40.8}, {116.3, 40.8}},

        {{-41.3, 49.2}, {-18.7, 49.2}},
        {{-11.3, 49.2}, {41.3, 49.2}},
        {{48.7, 49.2}, {106.3, 49.2}},
        {{113.7, 49.2}, {116.3, 49.2}},

        {{-41.3, 95.8}, {-18.7, 95.8}},
        {{-11.3, 95.8}, {41.3, 95.8}},
        {{48.7, 95.8}, {106.3, 95.8}},
        {{113.7, 95.8}, {116.3, 95.8}},

        {{-48.7, 104.2}, {123.7, 104.2}},

    };

};

#endif