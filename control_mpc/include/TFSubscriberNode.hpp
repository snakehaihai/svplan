#ifndef TF_SUBSCRIBER_NODE_HPP
#define TF_SUBSCRIBER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "Matrix.hpp"

class TFSubscriberNode : public rclcpp::Node {
public:
    TFSubscriberNode();

    std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const {
        return tfBuffer_;
    }

    Matrix Matrix_Read(const std::string &target_frame, const std::string &source_frame);

private:
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;
};

#endif // TF_SUBSCRIBER_NODE_HPP