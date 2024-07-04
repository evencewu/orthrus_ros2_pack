#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"

#include <xbox_interfaces/msg/xbox_control.hpp>
#include "orthrus_controller/interfaces/PinocchioInterfaces.hpp"

#include <iostream>
#include <mutex>

#include <Eigen/Dense>

namespace orthrus_controller
{
    class JoyInterface : public std::enable_shared_from_this<JoyInterface>
    {
    public:
        template <typename NodeType>
        JoyInterface(std::shared_ptr<NodeType> node)
        {
            node_ = node;

            xbox_sub_ = node->template create_subscription<xbox_interfaces::msg::XboxControl>("/xbox", 2, std::bind(&JoyInterface::JoyCallback, this, std::placeholders::_1));
        }

        void Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr);
        void JoyCallback(const xbox_interfaces::msg::XboxControl::SharedPtr msg);

        double RockerMapping(int input, int input_max, double output_max, int zero_zone);
        std::stringstream GetJoyTarget();

    private:
        std::mutex mtx_;
        //std::mutex mylock_;

        // Controller Interface
        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;
        
        OrthrusInterfaces orthrus_interfaces_self_;

        // ROS2
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        rclcpp::Subscription<xbox_interfaces::msg::XboxControl>::SharedPtr xbox_sub_;
        xbox_interfaces::msg::XboxControl xbox_msg_;
    };

}