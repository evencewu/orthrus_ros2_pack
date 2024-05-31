#include "orthrus_controller/visualization/OrthrusVisualization.hpp"

namespace orthrus_controller
{
    OrthrusVisualization::OrthrusVisualization(const rclcpp::Node::SharedPtr node,
                                               std::vector<std::string> joint_name)
        : joint_name_(joint_name)
    {
        RCLCPP_INFO(node->get_logger(), "OrthrusVisualization active.");
        joint_state_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        odom_publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    }

    OrthrusVisualization::OrthrusVisualization(const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                               std::vector<std::string> joint_name)
        : joint_name_(joint_name)
    {
        RCLCPP_INFO(node->get_logger(), "OrthrusVisualization active.");
        joint_state_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        odom_publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    }

    void OrthrusVisualization::update(rclcpp::Node::SharedPtr node)
    {
        joint_state_msg_.header.stamp = node->now();

        joint_state_msg_.header.frame_id = "body";

        std::vector<double> position = std::vector<double>(12);
        std::vector<double> velocity = std::vector<double>(12);
        std::vector<double> effort = std::vector<double>(12);

        for (int i = 0; i < 12; i++)
        {
            position[i] = 0;
            velocity[i] = 0;
            effort[i] = 0;
        }

        joint_state_msg_.name = joint_name_;
        joint_state_msg_.position = position;
        joint_state_msg_.velocity = velocity;
        joint_state_msg_.effort = effort;
        joint_state_publisher_->publish(joint_state_msg_);

        geometry_msgs::msg::TransformStamped tf_stamped;
        // orthrus_viewer_horizontal_pub_
        tf_stamped.header.stamp = node->now();

        tf_stamped.header.frame_id = "odom";
        tf_stamped.child_frame_id = "base";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;

        tf_stamped.transform.rotation.w = 1;
        tf_stamped.transform.rotation.x = 0;
        tf_stamped.transform.rotation.y = 0;
        tf_stamped.transform.rotation.z = 0;

        odom_msg_.transforms.clear();
        odom_msg_.transforms.push_back(tf_stamped);
        odom_publisher_->publish(odom_msg_);
    }

    void OrthrusVisualization::update(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    {
        joint_state_msg_.header.stamp = node->now();

        joint_state_msg_.header.frame_id = "body";

        std::vector<double> position = std::vector<double>(12);
        std::vector<double> velocity = std::vector<double>(12);
        std::vector<double> effort = std::vector<double>(12);

        for (int i = 0; i < 12; i++)
        {
            position[i] = 0;
            velocity[i] = 0;
            effort[i] = 0;
        }

        joint_state_msg_.name = joint_name_;
        joint_state_msg_.position = position;
        joint_state_msg_.velocity = velocity;
        joint_state_msg_.effort = effort;
        joint_state_publisher_->publish(joint_state_msg_);

        geometry_msgs::msg::TransformStamped tf_stamped;
        // orthrus_viewer_horizontal_pub_
        tf_stamped.header.stamp = node->now();

        tf_stamped.header.frame_id = "odom";
        tf_stamped.child_frame_id = "base";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;

        tf_stamped.transform.rotation.w = 1;
        tf_stamped.transform.rotation.x = 0;
        tf_stamped.transform.rotation.y = 0;
        tf_stamped.transform.rotation.z = 0;

        odom_msg_.transforms.clear();
        odom_msg_.transforms.push_back(tf_stamped);
        odom_publisher_->publish(odom_msg_);
    }
}
