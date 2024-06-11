#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"

#include <iostream>

namespace orthrus_controller
{
    class LeggedMpc
    {
    public:
        template <typename NodeType>
        LeggedMpc(std::shared_ptr<NodeType> node)
        {
            node_ = node;
        }

        void Init(std::shared_ptr<JointState> joint_ptr,
                  std::shared_ptr<OdomState> odom_state_,
                  std::shared_ptr<std::vector<TouchState>> touch_ptr,
                  std::shared_ptr<PinocchioInterface> pinocchio_ptr);

        void Update(rclcpp::Time time, rclcpp::Duration duration);

    private:
        std::shared_ptr<OdomState> odom_state_;
        std::shared_ptr<JointState> joint_state_;
        std::shared_ptr<std::vector<TouchState>> touch_state_;

        std::shared_ptr<PinocchioInterface> pinocchio_interface_; // Pinocchio接口

        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;
    };
}