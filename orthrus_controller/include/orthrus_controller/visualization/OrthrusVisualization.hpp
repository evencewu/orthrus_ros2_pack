#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_interface/controller_interface.hpp"

#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "orthrus_controller/interfaces/PinocchioInterface.hpp"

namespace orthrus_controller
{
    class OrthrusVisualization
    {
    public:
        template <typename NodeType>
        OrthrusVisualization(std::shared_ptr<NodeType> node, std::vector<std::string> joint_name) : joint_name_(joint_name)
        {
            RCLCPP_INFO(node->get_logger(), "OrthrusVisualization active.");
            joint_state_publisher_ = node->template create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            odom_publisher_ = node->template create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        }

        template <typename NodeType>
        void update(std::shared_ptr<NodeType> node)
        {
            joint_state_msg_.header.stamp = node->now();
            joint_state_msg_.header.frame_id = "body";

            // Direct initialization with zeros
            joint_state_msg_.position = std::vector<double>(12, 0.0);
            joint_state_msg_.velocity = std::vector<double>(12, 0.0);
            joint_state_msg_.effort = std::vector<double>(12, 0.0);

            joint_state_msg_.name = joint_name_;
            joint_state_publisher_->publish(joint_state_msg_);

            geometry_msgs::msg::TransformStamped tf_stamped;
            tf_stamped.header.stamp = node->now();
            tf_stamped.header.frame_id = "odom";
            tf_stamped.child_frame_id = "base";

            tf_stamped.transform.translation.x = 0.0;
            tf_stamped.transform.translation.y = 0.0;
            tf_stamped.transform.translation.z = 0.0;
            tf_stamped.transform.rotation.w = 1.0;
            tf_stamped.transform.rotation.x = 0.0;
            tf_stamped.transform.rotation.y = 0.0;
            tf_stamped.transform.rotation.z = 0.0;

            odom_msg_.transforms.clear();
            odom_msg_.transforms.push_back(tf_stamped);
            odom_publisher_->publish(odom_msg_);
        }

    private:
        std::vector<std::string> joint_name_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        sensor_msgs::msg::JointState joint_state_msg_;

        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odom_publisher_;
        tf2_msgs::msg::TFMessage odom_msg_;
    };
}