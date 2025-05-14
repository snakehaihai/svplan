#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gazebo
{
class OdomPlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
    {
        // Initialize ROS node
        this->ros_node = gazebo_ros::Node::Get(sdf);

        // Store the pointer to the model
        this->model = parent;

        // Setup odometry publisher
        odom_pub = this->ros_node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->ros_node);

        // Setup the update rate
        if (sdf->HasElement("update_rate")) {
            double update_rate = sdf->Get<double>("update_rate");
            this->update_period = 1.0 / update_rate;
        } else {
            this->update_period = 0.1;  // Default update rate 10Hz
        }

        // Initialize frame names
        if (sdf->HasElement("odom_frame")) {
            this->odom_frame = sdf->Get<std::string>("odom_frame");
        } else {
            this->odom_frame = "odom";
        }

        if (sdf->HasElement("base_frame")) {
            this->base_frame = sdf->Get<std::string>("base_frame");
        } else {
            this->base_frame = "car_base";
        }

        // Connect to the world update event
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&OdomPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
        common::Time current_time = this->model->GetWorld()->SimTime();
        if ((current_time - last_update_time).Double() >= update_period) {
            last_update_time = current_time;

            // Get the current pose of the car_base
            auto pose = this->model->WorldPose();

            // Get the velocity of the car_base
            auto linear_velocity = this->model->RelativeLinearVel();
            auto angular_velocity = this->model->RelativeAngularVel();

            // Publish odometry message
            nav_msgs::msg::Odometry odom;

            odom.header.stamp = this->ros_node->now();
            odom.header.frame_id = odom_frame;
            odom.child_frame_id = base_frame;
            odom.pose.pose.position.x = pose.Pos().X();
            odom.pose.pose.position.y = pose.Pos().Y();
            odom.pose.pose.position.z = pose.Pos().Z();
            odom.pose.pose.orientation.x = pose.Rot().X();
            odom.pose.pose.orientation.y = pose.Rot().Y();
            odom.pose.pose.orientation.z = pose.Rot().Z();
            odom.pose.pose.orientation.w = pose.Rot().W();

            // Set the velocity in the odom message
            odom.twist.twist.linear.x = linear_velocity.X();
            odom.twist.twist.linear.y = linear_velocity.Y();
            odom.twist.twist.linear.z = linear_velocity.Z();
            odom.twist.twist.angular.x = angular_velocity.X();
            odom.twist.twist.angular.y = angular_velocity.Y();
            odom.twist.twist.angular.z = angular_velocity.Z();

            odom_pub->publish(odom);

            // Publish TF transform
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = odom.header.stamp;
            transformStamped.header.frame_id = odom_frame;
            transformStamped.child_frame_id = base_frame;
            transformStamped.transform.translation.x = pose.Pos().X();
            transformStamped.transform.translation.y = pose.Pos().Y();
            transformStamped.transform.translation.z = pose.Pos().Z();
            transformStamped.transform.rotation = odom.pose.pose.orientation;

            tf_broadcaster_->sendTransform(transformStamped);
        }
    }

private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    gazebo_ros::Node::SharedPtr ros_node;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string odom_frame;
    std::string base_frame;
    common::Time last_update_time;
    double update_period;
};

GZ_REGISTER_MODEL_PLUGIN(OdomPlugin)
}
