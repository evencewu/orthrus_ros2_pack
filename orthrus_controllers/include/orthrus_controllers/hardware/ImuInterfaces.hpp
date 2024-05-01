#pragma once

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/imu.hpp>

namespace orthrus_control
{
    class ImuInterfaces
    {
    public:
        ImuInterfaces(const rclcpp::Node::SharedPtr &node);

    private:
        rclcpp::Node::SharedPtr node_;

        // 接收imu消息并发送odom信息
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orthrus_imu_sub_;
        sensor_msgs::msg::Imu orthrus_imu_msg_;

        void OrthrusImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    };
}