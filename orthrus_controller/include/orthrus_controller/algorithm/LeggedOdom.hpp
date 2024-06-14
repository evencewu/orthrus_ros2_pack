#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_interface/controller_interface.hpp"

#include "orthrus_controller/interfaces/OrthrusInterfaces.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

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

        void Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr);
        void Update(rclcpp::Time time, rclcpp::Duration duration);
        void calibration(rclcpp::Time time, rclcpp::Duration duration);

        Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond quat);
    private:
        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;
    };
}