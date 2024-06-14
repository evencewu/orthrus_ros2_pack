#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"

#include <xbox_interfaces/msg/xbox_control.hpp>

#include <iostream>

#include <Eigen/Dense>

namespace orthrus_controller
{
    class JoyInterface
    {
    public:
        template <typename NodeType>
        JoyInterface(std::shared_ptr<NodeType> node)
        {
            node_ = node;

            xbox_sub_ = node->template create_subscription<xbox_interfaces::msg::XboxControl>("/xbox", 2, std::bind(&JoyInterface::JoyCallback, this, std::placeholders::_1));
        }

        void Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr);

    private:
        void JoyCallback(const xbox_interfaces::msg::XboxControl::SharedPtr msg);

        // Controller Interface
        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;

        // ROS2
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        rclcpp::Subscription<xbox_interfaces::msg::XboxControl>::SharedPtr xbox_sub_;
        xbox_interfaces::msg::XboxControl xbox_msg_;
    };

}