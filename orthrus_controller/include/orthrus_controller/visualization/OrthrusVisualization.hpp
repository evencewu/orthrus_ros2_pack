#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_interface/controller_interface.hpp"

#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "orthrus_controller/interfaces/OrthrusParma.hpp"
#include "orthrus_controller/interfaces/PinocchioInterface.hpp"

namespace orthrus_controller
{
    class OrthrusVisualization : public std::enable_shared_from_this<OrthrusVisualization>
    {
    public:
        template <typename NodeType>
        OrthrusVisualization(std::shared_ptr<NodeType> node, std::vector<std::string> joint_name) : joint_name_(joint_name)
        {
            node_ = node;
            RCLCPP_INFO(node->get_logger(), "OrthrusVisualization active.");
            joint_state_publisher_ = node->template create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            odom_publisher_ = node->template create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        }

        void Init(std::shared_ptr<JointParma> joint_parma_ptr);
        void update(rclcpp::Time time);
    private:
        std::shared_ptr<JointParma> joint_parma_;

        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        std::vector<std::string> joint_name_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        sensor_msgs::msg::JointState joint_state_msg_;

        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odom_publisher_;
        tf2_msgs::msg::TFMessage odom_msg_;
    };
}