#include "TFSubscriberNode.hpp"
#include "A_Star_Node.hpp"
#include "TrajectoryOpimizationNode.hpp"
#include "SDFOpimizationNode.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto tf_subscriber_node = std::make_shared<TFSubscriberNode>();
    auto route_planning_node = std::make_shared<A_Star_Node>(tf_subscriber_node);
    auto trajectory_opimization_node = std::make_shared<Trajectory_Opimization_Node>(tf_subscriber_node);
    auto sdf_opimization_node = std::make_shared<SDF_Opimization_Node>(tf_subscriber_node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(tf_subscriber_node);
    executor.add_node(route_planning_node);
    executor.add_node(trajectory_opimization_node);
    executor.add_node(sdf_opimization_node);
    executor.spin();  // 启动并运行所有节点
    rclcpp::shutdown();

    return 0;
}
