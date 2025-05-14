#include "A_Star_Node.hpp"

A_Star_Node::A_Star_Node(std::shared_ptr<TFSubscriberNode> tf_subscriber_node)
    : Node("control_node"), tf_subscriber_node_(tf_subscriber_node) {
    // 初始化订阅者
    point_cloud_msg_.subscribe(this, "/gazebo_laser_controller/out");
    goal_msg_.subscribe(this, "/goal_pose");

   // 使用别名创建同步器实例
    sync_ = std::make_shared<MySynchronizer>(SyncPolicy(100), point_cloud_msg_, goal_msg_);
    sync_->registerCallback(std::bind(&A_Star_Node::Callback, this, std::placeholders::_1, std::placeholders::_2));

    // 初始化发布者
    Filter_Publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/car/Filter", 10);
    A_Star_Path_Rviz_Publisher_ = this->create_publisher<nav_msgs::msg::Path>("A_Star_Planned_Path_Rviz", 10);
    A_Star_Path_Publisher_ = this->create_publisher<astar_msgs::msg::AStarPathArray>("A_Star_Planned_Path", 10);
    A_Star_Map_Publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("A_Star_marker_array", 10);
    Road_Side_Publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("Road_Side", 10);
    config_yaml_path = "/root/PersonalData/Program/multi-axle-all-wheel-steering-vehicles_ws/src/route_planning/config/route_planning_config.yaml";

}

void A_Star_Node::Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & point_cloud_msg, const geometry_msgs::msg::PoseStamped::ConstSharedPtr & goal_msg)
{
    Lidar_Car_Matrix = tf_subscriber_node_ -> Matrix_Read("car_base", "laser_center");  // 激光雷达 ->  车辆 变换矩阵获取(放在回调函数最开始，保证和初始数据时间戳尽量同步)
    Odom_Car_Matrix = tf_subscriber_node_ -> Matrix_Read("car_base", "odom");           // 世界    ->  车辆 变换矩阵获取(放在回调函数最开始，保证和初始数据时间戳尽量同步)
    Car_Odom_Matrix = tf_subscriber_node_ -> Matrix_Read("odom", "car_base");           // 车辆    ->  世界 变换矩阵获取(放在回调函数最开始，保证和初始数据时间戳尽量同步)
    
    config = YAML::LoadFile(config_yaml_path);
    // 通过话题读取点云信息
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_msg, *cloud);
    // 第一次过滤：保留 0<X<20 (m)的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_PassThrough_X(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0, INFINITY);
    pass_x.filter(*filtered_cloud_PassThrough_X);
    // 第三次过滤：保留-0.04<Z<0.5 (m)的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_PassThrough_Z(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(filtered_cloud_PassThrough_X);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(-0.35, 0.5);
    pass_z.filter(*filtered_cloud_PassThrough_Z);
    // 第四次过滤：体素网格滤波
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_VoxelGrid(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(filtered_cloud_PassThrough_Z);
    vg.setLeafSize(0.02f, 0.02f, 0.02f);
    vg.filter(*filtered_cloud_VoxelGrid);

    //使用CPU聚类
    //创建k-d树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered_cloud_VoxelGrid);
    //创建欧氏聚类提取器
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.1); // 10cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_cloud_VoxelGrid);
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 遍历每个簇,计算外包长方体
    Eigen::Matrix3Xd Obstacle_Point_AABB;  //点云外包长方体俯视投影，激光雷达坐标
    Obstacle_Point_AABB.resize(3, cluster_indices.size() * 4); // 预分配空间

    int point_index = 0;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        // 提取当前簇的点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices)
            cluster->push_back((*filtered_cloud_VoxelGrid)[idx]);

        // 计算并输出每个簇的外包矩形和几何中心
        Eigen::Vector4f min_point, max_point;
        pcl::getMinMax3D(*cluster, min_point, max_point);

        // 计算外包长方体的4个角点俯视投影
        for (int i = 0; i < 4; i++) {
            Obstacle_Point_AABB.col(point_index++) <<   ((i & 1) ? max_point[0] : min_point[0]),
                                                        ((i & 2) ? max_point[1] : min_point[1]),
                                                        0.0;  // Z坐标为0，因为是俯视投影
        }
    }
    // 创建一个 4xN 的矩阵来存储点云数据（齐次坐标）
    Eigen::MatrixXd Points_Lidar_Matrix(4, Obstacle_Point_AABB.cols());
    for (int i = 0; i < Obstacle_Point_AABB.cols(); ++i) {
        const auto& point = Obstacle_Point_AABB.col(i);
        Points_Lidar_Matrix(0, i) = point(0);  // X 坐标
        Points_Lidar_Matrix(1, i) = point(1);  // Y 坐标
        Points_Lidar_Matrix(2, i) = point(2);  // Z 坐标
        Points_Lidar_Matrix(3, i) = 1.0;      // 齐次坐标的 1
    }
    Eigen::MatrixXd Lidar_Car_Rotation_Translation_Matrix = Lidar_Car_Matrix.Rotation_Translation_Read();
    // 进行矩阵变换
    Eigen::MatrixXd Obstacle_Transformed_Points_Lidar_Matrix = Lidar_Car_Rotation_Translation_Matrix * Points_Lidar_Matrix;

    // 获取点云中的点数量
    int num_points = filtered_cloud_VoxelGrid->points.size();
    // 创建一个 4xN 的矩阵来存储点云数据（齐次坐标）
    Points_Lidar_Matrix = Eigen::MatrixXd::Zero(4, num_points);
    for (int i = 0; i < num_points; ++i) {
        const auto& point = filtered_cloud_VoxelGrid->points[i];
        Points_Lidar_Matrix(0, i) = point.x;  // X 坐标
        Points_Lidar_Matrix(1, i) = point.y;  // Y 坐标
        Points_Lidar_Matrix(2, i) = 0;        // Z 坐标
        Points_Lidar_Matrix(3, i) = 1.0;      // 齐次坐标的 1
    }
    // 进行矩阵变换
    Eigen::MatrixXd Transformed_Points_Lidar_Matrix = Lidar_Car_Rotation_Translation_Matrix * Points_Lidar_Matrix;

    // 获取目标点位置并转换到车辆坐标系
    Eigen::MatrixXd Odom_Car_Rotation_Translation_Matrix = Odom_Car_Matrix.Rotation_Translation_Read();
    Eigen::VectorXd Point_Destination(4);
    Point_Destination << goal_msg->pose.position.x,
                        goal_msg->pose.position.y,
                        goal_msg->pose.position.z,
                        1.0;
    Eigen::VectorXd Transformed_Point_Destination = Odom_Car_Rotation_Translation_Matrix * Point_Destination;
    double Transformed_Point_Destination_X = Transformed_Point_Destination[0];
    double Transformed_Point_Destination_Y = Transformed_Point_Destination[1];
    // 定义遍历的区域和分辨率（以米为单位）
    double x_min = config["x_min"].as<double>();
    double x_max = config["x_max"].as<double>();
    double y_min = config["y_min"].as<double>();
    double y_max = config["y_max"].as<double>();
    x_min = fmax(x_min, fmin(Transformed_Point_Destination_X, 0)) - 5;
    x_max = fmin(x_max, fmax(Transformed_Point_Destination_X, 0)) + 5;
    y_min = fmax(y_min, fmin(Transformed_Point_Destination_Y, 0)) - 5; 
    y_max = fmin(y_max, fmax(Transformed_Point_Destination_Y, 0)) + 5;
    // 定义栅格地图的分辨率
    const double grid_resolution_meters = config["a_star_resolution_meters"].as<double>();                   // 每个栅格的边长

    // 根据分辨率计算栅格地图的宽度和高度（单位：栅格）
    int grid_width = static_cast<int>(std::round((x_max - x_min) / grid_resolution_meters)) + 1;
    int grid_height = static_cast<int>(std::round((y_max - y_min) / grid_resolution_meters)) + 1;
    // 初始化栅格地图
    std::vector<std::vector<int>> grid_map(grid_height, std::vector<int>(grid_width, 0));
    // 将点云数据映射到栅格地图中
    for (int i = 0; i < Transformed_Points_Lidar_Matrix.cols(); ++i) {
        double Transformed_Point_Lidar_X = Transformed_Points_Lidar_Matrix(0, i);
        double Transformed_Point_Lidar_Y = Transformed_Points_Lidar_Matrix(1, i);

        // 检查点是否在网格的范围内
        if (Transformed_Point_Lidar_X >= x_min && Transformed_Point_Lidar_X < x_max &&
            Transformed_Point_Lidar_Y >= y_min && Transformed_Point_Lidar_Y < y_max) {
            
            // 计算栅格索引
            int grid_x = static_cast<int>(std::round((Transformed_Point_Lidar_X - x_min) / grid_resolution_meters));
            int grid_y = static_cast<int>(std::round((Transformed_Point_Lidar_Y - y_min) / grid_resolution_meters));

            // 更新 grid_map，标记为不可通行
            if(grid_x>0 && grid_x<grid_width && grid_y>0 && grid_y < grid_height)
                grid_map[grid_y][grid_x] = 1;
        }
    }

    //将道路坐标从世界坐标系转到车辆坐标系
    Eigen::MatrixXd Points_Road_Matrix(4, roads.size() * 2);
    Points_Road_Matrix.row(2).setOnes();
    Points_Road_Matrix.row(3).setOnes();
    for (size_t i = 0; i < roads.size(); ++i) {
        auto& [start, end, width] = roads[i];
        Points_Road_Matrix(0, 2 * i) = start[0];     // 起点的 X
        Points_Road_Matrix(1, 2 * i) = start[1];     // 起点的 Y
        Points_Road_Matrix(0, 2 * i + 1) = end[0];   // 终点的 X
        Points_Road_Matrix(1, 2 * i + 1) = end[1];   // 终点的 Y
    }
    Eigen::MatrixXd Transformed_Points_Road = Odom_Car_Rotation_Translation_Matrix * Points_Road_Matrix;
    std::vector<std::tuple<std::array<double, 2>, std::array<double, 2>, double>> transformed_roads;
    transformed_roads.resize(roads.size());
    for (size_t i = 0; i < transformed_roads.size(); ++i) {
        auto& [start, end, width] = transformed_roads[i];
        auto& [start_, end_, width_] = roads[i];
        start[0] = Transformed_Points_Road(0, 2 * i);
        start[1] = Transformed_Points_Road(1, 2 * i);
        end[0] = Transformed_Points_Road(0, 2 * i + 1);
        end[1] = Transformed_Points_Road(1, 2 * i + 1);
        width = width_;
    }
    
    //将道路映射到地图中
    for(int w=0; w<grid_width; w++)
    {
        for(int h=0; h<grid_height; h++)
        {
            // 计算栅格角点位置
            double x_1 = (w + std::round(x_min / grid_resolution_meters)) * grid_resolution_meters - grid_resolution_meters / 2.0;
            double x_2 = (w + std::round(x_min / grid_resolution_meters)) * grid_resolution_meters + grid_resolution_meters / 2.0;
            double y_1 = (h + std::round(y_min / grid_resolution_meters)) * grid_resolution_meters - grid_resolution_meters / 2.0;
            double y_2 = (h + std::round(y_min / grid_resolution_meters)) * grid_resolution_meters + grid_resolution_meters / 2.0;
            bool onroad = false;
            for (size_t i = 0; i < transformed_roads.size(); i++) 
            {
                auto& [start, end, width] = transformed_roads[i];
                if (isPointOnRoad({x_1, y_1}, {start[0], start[1]}, {end[0], end[1]}, width) && 
                    isPointOnRoad({x_1, y_2}, {start[0], start[1]}, {end[0], end[1]}, width) && 
                    isPointOnRoad({x_2, y_1}, {start[0], start[1]}, {end[0], end[1]}, width) && 
                    isPointOnRoad({x_2, y_2}, {start[0], start[1]}, {end[0], end[1]}, width))
                {
                    onroad = true;
                    break;
                }
            }
            if(onroad == false)
            {
                grid_map[h][w] = 1;
            }
        }
    }

    std::pair<int, int> start(
        static_cast<int>(std::round((0 - x_min) / grid_resolution_meters)),                                                                             // 将实际坐标转换为栅格索引
        static_cast<int>(std::round((0 - y_min) / grid_resolution_meters))
    );

    std::pair<int, int> goal(
        static_cast<int>(std::round((Transformed_Point_Destination_X - x_min) / grid_resolution_meters)),      // 将实际坐标转换为栅格索引
        static_cast<int>(std::round((Transformed_Point_Destination_Y - y_min) / grid_resolution_meters))
    );
    A_Star_Path = AStarSearch(grid_map, start, goal, grid_resolution_meters);
    
    Eigen::MatrixXd Points_Road_Side_Matrix(4, roads_side.size() * 2);
    Points_Road_Side_Matrix.row(2).setOnes();
    Points_Road_Side_Matrix.row(3).setOnes();
    for (size_t i = 0; i < roads_side.size(); ++i) {
        auto& [start, end] = roads_side[i];
        Points_Road_Side_Matrix(0, 2 * i) = start[0];     // 起点的 X
        Points_Road_Side_Matrix(1, 2 * i) = start[1];     // 起点的 Y
        Points_Road_Side_Matrix(0, 2 * i + 1) = end[0];   // 终点的 X
        Points_Road_Side_Matrix(1, 2 * i + 1) = end[1];   // 终点的 Y
    }
    Eigen::MatrixXd Transformed_Points_Road_Side = Odom_Car_Rotation_Translation_Matrix * Points_Road_Side_Matrix;
    std::vector<std::tuple<std::array<double, 2>, std::array<double, 2>>> transformed_roads_side;
    transformed_roads_side.resize(roads_side.size());
    for (size_t i = 0; i < transformed_roads_side.size(); ++i) {
        auto& [start, end] = transformed_roads_side[i];
        start[0] = Transformed_Points_Road_Side(0, 2 * i);
        start[1] = Transformed_Points_Road_Side(1, 2 * i);
        end[0] = Transformed_Points_Road_Side(0, 2 * i + 1);
        end[1] = Transformed_Points_Road_Side(1, 2 * i + 1);
    }

    auto result = detectRoadSegments(x_min, x_max, y_min, y_max, transformed_roads_side);
    

    // 检查路径是否为空
    astar_msgs::msg::AStarPathArray AStarPathArray;
    if (A_Star_Path.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Path planning failed: no valid path found.");
    } else {
            AStarPathArray.header.stamp = point_cloud_msg->header.stamp;
            AStarPathArray.header.frame_id = "car_base";
            astar_msgs::msg::AStarPath AStarPath;
            for(int i=0; i<A_Star_Path.size() - 1; i++)
            {
                AStarPath.position.x = (A_Star_Path[i].position[0] + std::round(x_min / grid_resolution_meters)) * grid_resolution_meters;
                AStarPath.position.y = (A_Star_Path[i].position[1] + std::round(y_min / grid_resolution_meters)) * grid_resolution_meters;
                AStarPath.position.z = A_Star_Path[i].yaw;
                AStarPathArray.paths.push_back(AStarPath);
            }
            double roll, pitch, yaw;
            tf2::Quaternion tf_arrow_world, tf_arrow_car, tf_Odom_to_car;
            tf2::fromMsg(goal_msg->pose.orientation, tf_arrow_world);
            tf2::fromMsg(Odom_Car_Matrix.Quaternion_Read(), tf_Odom_to_car);
            tf_arrow_car = tf_Odom_to_car * tf_arrow_world;
            tf2::Matrix3x3(tf_arrow_car).getRPY(roll, pitch, yaw);
            AStarPath.position.x = (A_Star_Path[A_Star_Path.size() - 1].position[0] + std::round(x_min / grid_resolution_meters)) * grid_resolution_meters;
            AStarPath.position.y = (A_Star_Path[A_Star_Path.size() - 1].position[1] + std::round(y_min / grid_resolution_meters)) * grid_resolution_meters;;
            AStarPath.position.z = yaw;
            AStarPathArray.paths.push_back(AStarPath);

            for(int i=0; i<Obstacle_Transformed_Points_Lidar_Matrix.cols(); i++)
            {
                geometry_msgs::msg::Vector3 point;
                point.x = Obstacle_Transformed_Points_Lidar_Matrix(0, i);
                point.y = Obstacle_Transformed_Points_Lidar_Matrix(1, i);
                point.z = 0;
                AStarPathArray.obstacle_points.push_back(point);
            }

            for (const auto& segment : result) 
            {
                auto [start, end] = segment;
                geometry_msgs::msg::Vector3 point;
                double x1 = start[0], y1 = start[1];
                double x2 = end[0], y2 = end[1];
                // 计算线段长度
                double distance = std::hypot(x2 - x1, y2 - y1);
                double num_points = std::ceil(distance / config["Road_Side_Point_interval"].as<double>()); // 计算需要多少个点
                double t_step = 1.0 / num_points; // 每个点在线段上的参数步长
                for (int i = 0; i <= num_points; ++i) 
                {
                    geometry_msgs::msg::Vector3 point;
                    point.x = x1 + t_step * i * (x2 - x1);
                    point.y = y1 + t_step * i * (y2 - y1);
                    point.z = 0;
                    AStarPathArray.obstacle_points.push_back(point);
                }
            }
            // 发布路径
            A_Star_Path_Publisher_->publish(AStarPathArray);
    }

    //RVIZ可视化
    if(config["Publish_Filter_Lidar_Point"].as<bool>())
    {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*filtered_cloud_VoxelGrid, msg);
        msg.header.stamp = point_cloud_msg->header.stamp;
        msg.header.frame_id = point_cloud_msg->header.frame_id;
        // 发布点云
        Filter_Publisher->publish(msg);
    }

    if(config["Publish_Road_Side_Point"].as<bool>())
    {
        // 发布道路边界点云
        sensor_msgs::msg::PointCloud2 road_side_cloud_msg;
        road_side_cloud_msg.header.frame_id = "car_base";  // 设置坐标系
        road_side_cloud_msg.header.stamp = point_cloud_msg->header.stamp;

        road_side_cloud_msg.height = 1;
        road_side_cloud_msg.width = AStarPathArray.obstacle_points.size();
        road_side_cloud_msg.is_dense = true;
        road_side_cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(road_side_cloud_msg);
        modifier.setPointCloud2Fields(3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);

        sensor_msgs::PointCloud2Iterator<float> iter_x(road_side_cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(road_side_cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(road_side_cloud_msg, "z");

        for (const auto& point : AStarPathArray.obstacle_points)
        {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = -1.2;
            ++iter_x; ++iter_y; ++iter_z;
        }
        Road_Side_Publisher_->publish(road_side_cloud_msg);
    }
    if(config["Publish_A_Star_Map"].as<bool>())
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "odom";
        delete_marker.header.stamp = point_cloud_msg->header.stamp;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
        A_Star_Map_Publisher_->publish(marker_array);
        marker_array.markers.clear(); // 清空数组以便添加新标记

        // 将map点转移到世界坐标系，因为frame_id = "car_base"时，rviz总查不到TF，时间戳超了，有BUG，所以手动转换
        geometry_msgs::msg::Quaternion Quaternion_Car = Car_Odom_Matrix.Quaternion_Read();
        Eigen::MatrixXd Car_Odom_Rotation_Translation_Matrix = Car_Odom_Matrix.Rotation_Translation_Read();
        int marker_id = 0;
        for (int y = 0; y < grid_height; ++y) {
            for (int x = 0; x < grid_width; ++x) {
                // 创建一个 Marker
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "odom";  // 使用世界坐标系
                marker.header.stamp = point_cloud_msg->header.stamp;
                marker.ns = "grid";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                Eigen::VectorXd Point_Map(4);
                Point_Map <<    (x + std::round(x_min / grid_resolution_meters)) * grid_resolution_meters,
                                (y + std::round(y_min / grid_resolution_meters)) * grid_resolution_meters,
                                0,
                                1.0;

                Eigen::VectorXd Transformed_Point_Map = Car_Odom_Rotation_Translation_Matrix * Point_Map;
                marker.pose.position.x = Transformed_Point_Map(0, 0);
                marker.pose.position.y = Transformed_Point_Map(1, 0); 
                marker.pose.position.z = 0;
                marker.pose.orientation = Quaternion_Car;

                // 设置标记大小
                marker.scale.x = grid_resolution_meters;
                marker.scale.y = grid_resolution_meters;
                marker.scale.z = -0.1;  // 设置 Z 方向的厚度为 0.1

                // 根据网格的位置设置颜色
                if (x == start.first && y == start.second) {
                    // 起点为白色
                    marker.color.r = 1.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 1.0f;
                    marker.color.a = 1.0f;
                } 
                else if (x == goal.first && y == goal.second) {
                    // 终点为黄色
                    marker.color.r = 1.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 0.0f;
                    marker.color.a = 1.0f;
                } 
                else if (grid_map[y][x] == 1) {
                    // 有障碍物的格子为红色
                    marker.color.r = 1.0f;
                    marker.color.g = 0.0f;
                    marker.color.b = 0.0f;
                    marker.color.a = 1.0f;
                } 
                else {
                    // 无障碍物的格子为绿色
                    marker.color.r = 0.0f;
                    marker.color.g = 1.0f;
                    marker.color.b = 0.0f;
                    marker.color.a = 0.5f;
                }

                // 将 Marker 添加到 MarkerArray 中
                marker_array.markers.push_back(marker);
            }
        }
        // 发布 MarkerArray
        A_Star_Map_Publisher_->publish(marker_array);
    }
    if(config["Publish_A_star_Path"].as<bool>())
    {
        if (!A_Star_Path.empty())
        {
            // 创建一个Path消息
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = point_cloud_msg->header.stamp;
            path_msg.header.frame_id = "car_base";  // 使用里程计的坐标系
            for(auto & path : A_Star_Path)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = point_cloud_msg->header.stamp;
                pose.header.frame_id = "car_base";
                pose.pose.position.x = (path.position[0] + std::round(x_min / grid_resolution_meters)) * grid_resolution_meters;
                pose.pose.position.y = (path.position[1] + std::round(y_min / grid_resolution_meters)) * grid_resolution_meters;
                pose.pose.position.z = -1.5;  // 假设路径在地面上，z 坐标为 0

                pose.pose.orientation.w = 1.0;  // 无旋转，仅设置为单位四元数
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = 0.0;

                path_msg.poses.push_back(pose);
            }
            // 发布路径
            A_Star_Path_Rviz_Publisher_->publish(path_msg);
        }
    }
}

// 判断点是否在道路上
bool A_Star_Node::isPointOnRoad(const std::array<double, 2>& point, const std::array<double, 2>& start, const std::array<double, 2>& end, double width)
{
    double px = point[0];
    double py = point[1];
    double sx = start[0];
    double sy = start[1];
    double ex = end[0];
    double ey = end[1];

    // 计算线段向量和点向量
    double vx = ex - sx;
    double vy = ey - sy;
    double wx = px - sx;
    double wy = py - sy;

    // 计算点到线段的最近点
    double c1 = wx * vx + wy * vy;
    if (c1 <= 0)
        return sqrt(wx * wx + wy * wy) <= width / 2; // Closest to start point

    double c2 = vx * vx + vy * vy;
    if (c2 <= c1)
        return sqrt((px - ex) * (px - ex) + (py - ey) * (py - ey)) <= width / 2; // Closest to end point

    double b = c1 / c2;
    double qx = sx + b * vx;
    double qy = sy + b * vy;

    // 计算点到最近点的距离
    return sqrt((px - qx) * (px - qx) + (py - qy) * (py - qy)) <= width / 2;
}

// AStarSearch 函数，返回 std::vector<A_Star_Path>
std::vector<A_Star_Path_> A_Star_Node::AStarSearch(const std::vector<std::vector<int>>& grid_map, const std::pair<int, int>& start, const std::pair<int, int>& goal, double grid_resolution_meters) 
{
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_list;
    std::vector<std::vector<bool>> closed_list(grid_map.size(), std::vector<bool>(grid_map[0].size(), false));
    std::vector<A_Star_Path_> a_star_path;

    open_list.emplace(start.first, start.second, 0.0f, std::hypot(goal.first - start.first, goal.second - start.second), -1, -1);

    std::vector<std::pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1},
                                                   {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};  // 8个方向

    std::map<std::pair<int, int>, std::pair<int, int>> came_from;

    while (!open_list.empty()) {
        AStarNode current = open_list.top();
        open_list.pop();

        if (current.x == goal.first && current.y == goal.second) {
            // 到达目标，构造路径
            std::pair<int, int> current_pos = {current.x, current.y};
            std::vector<std::pair<int, int>> path;
            while (current_pos != start) {
                path.push_back(current_pos);
                current_pos = came_from[current_pos];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());

            // 计算并存储路径点和yaw角
            for (size_t i = 0; i < path.size(); ++i) {
                A_Star_Path_ a_star_node;
                a_star_node.position = Eigen::Vector2d(path[i].first, path[i].second);
                if (i == 0) {
                    a_star_node.yaw = 0;
                } else {
                    a_star_node.yaw = atan2(path[i].second - path[i-1].second, path[i].first - path[i-1].first);
                }

                a_star_path.push_back(a_star_node);
            }

            return a_star_path;
        }

        closed_list[current.y][current.x] = true;

        for (const auto& dir : directions) {
            int new_x = current.x + dir.first;
            int new_y = current.y + dir.second;

            if (new_x >= 0 && new_x < grid_map[0].size() && new_y >= 0 && new_y < grid_map.size() &&
                !closed_list[new_y][new_x] && grid_map[new_y][new_x] == 0) {
                float new_g_cost = current.g_cost + std::hypot(dir.first, dir.second);
                float new_h_cost = std::hypot(goal.first - new_x, goal.second - new_y);

                if (came_from.find({new_x, new_y}) == came_from.end()) {
                    came_from[{new_x, new_y}] = {current.x, current.y};
                    open_list.emplace(new_x, new_y, new_g_cost, new_h_cost, current.x, current.y);
                }
            }
        }
    }

    // 如果没有找到路径，返回空路径
    return a_star_path;
}
