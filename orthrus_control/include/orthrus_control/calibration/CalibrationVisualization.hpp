#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


#include "orthrus_control/OrthrusControlVariable.hpp"

namespace orthrus_control
{
    class CalibrationVisualization
    {
    public:
        template <typename NodeType>
        CalibrationVisualization(std::shared_ptr<NodeType> node)
        {
            node_ = node;
            leg_imu_publisher_ = node->template create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        }

        void Init(std::shared_ptr<OrthrusControlVariable> variable);

        void Update(std::vector<double>);

    private:
        std::shared_ptr<OrthrusControlVariable> variable_;

        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        std::vector<std::string> imu_names = {"LF_IMU", "LH_IMU", "RF_IMU", "RH_IMU"};

        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr leg_imu_publisher_;
        tf2_msgs::msg::TFMessage leg_imu_msg_;
    };
}