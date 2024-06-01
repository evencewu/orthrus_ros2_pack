#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <iostream>

#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"

namespace orthrus_controller
{
    class PinocchioInterface
    {
    public:
        PinocchioInterface(const rclcpp::Node::SharedPtr node);
        PinocchioInterface(const rclcpp_lifecycle::LifecycleNode::SharedPtr node);

        void Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr node);

    private:
        pinocchio::Model model_;
        pinocchio::Data data_;

    };
}