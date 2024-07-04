#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"

#include "orthrus_controller/interfaces/PinocchioInterfaces.hpp"

#include <iostream>

namespace orthrus_controller
{
    class LeggedTouch
    {
    public:
        template <typename NodeType>
        LeggedTouch(std::shared_ptr<NodeType> node)
        {
            node_ = node;
        }
        void Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr,
                  std::shared_ptr<PinocchioInterfaces> pinocchio_ptr);

        

    private:
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;
        std::shared_ptr<PinocchioInterfaces> pinocchio_interfaces_; // Pinocchio接口

    };

}