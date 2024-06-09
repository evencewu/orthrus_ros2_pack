#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_interface/controller_interface.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "orthrus_controller/interfaces/OrthrusParma.hpp"

namespace orthrus_controller
{
    class LeggedOdom
    {
    public:
        template <typename NodeType>
        LeggedOdom(std::shared_ptr<NodeType> node)
        {
            node_ = node;
            RCLCPP_INFO(node->get_logger(), "Legged odom active.");
        }

        void Init(std::shared_ptr<OdomState> odom_ptr,
                  std::shared_ptr<std::vector<TouchState>> touch_ptr);
        void Update(rclcpp::Time time);

        Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond quat);

        std::shared_ptr<OdomState> odom_state_;
        std::shared_ptr<std::vector<TouchState>> touch_state_;

    private:
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;
    };
}