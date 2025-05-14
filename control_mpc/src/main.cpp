#include "TFSubscriberNode.hpp"
#include "Control_MPC.hpp"
#include "MPC.hpp"
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto tf_subscriber_node = std::make_shared<TFSubscriberNode>();
    auto control_node = std::make_shared<ControlNode>();
    auto mpc_node = std::make_shared<MpcNode>(control_node, tf_subscriber_node);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(control_node);
    executor.add_node(mpc_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}