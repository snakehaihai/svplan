#include "TFSubscriberNode.hpp"

TFSubscriberNode::TFSubscriberNode() : Node("tf_subscriber_node") {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);

    // Set timeout for waiting transform to become available
    auto timeout = std::chrono::seconds(5);
    auto start_time = this->get_clock()->now();

    // Wait until the transform from odom to laser_center becomes available
    while (rclcpp::ok()) {
        if (this->get_clock()->now() - start_time > timeout) {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for transform from TF");
            break;
        }
        if (tfBuffer_->canTransform("odom", "laser_center", tf2::TimePointZero, std::chrono::milliseconds(100))) {
            break; // Successfully obtained the transform, exit loop
        }
        // Sleep briefly before retrying
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
}

Matrix TFSubscriberNode::Matrix_Read(const std::string &target_frame, const std::string &source_frame)
{
    Matrix Transformation_Matrix;
    if (tfBuffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, std::chrono::milliseconds(100))) {
        try {
            auto transformStamped = tfBuffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            Transformation_Matrix.setFromTF(transformStamped);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s after becoming available: %s", target_frame, source_frame, ex.what());
        }
    }

    return Transformation_Matrix;
}
