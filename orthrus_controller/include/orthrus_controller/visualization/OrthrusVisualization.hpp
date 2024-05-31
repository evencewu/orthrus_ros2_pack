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
        OrthrusVisualization(const rclcpp::Node::SharedPtr node, std::vector<std::string> joint_name);
        OrthrusVisualization(const rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::vector<std::string> joint_name);

        void update(rclcpp::Node::SharedPtr node);
        void update(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
    private:
        std::vector<std::string> joint_name_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        sensor_msgs::msg::JointState joint_state_msg_;

        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odom_publisher_;
        tf2_msgs::msg::TFMessage odom_msg_;
    };
}