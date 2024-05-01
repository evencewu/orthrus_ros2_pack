#include "orthrus_controllers/interface/ImuInterfaces.hpp"

namespace orthrus_control
{
    ImuInterfaces::ImuInterfaces(const rclcpp::Node::SharedPtr &node) : node_(node)
    {
        orthrus_imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("/orthrus_interface/imu", 10, std::bind(&OrthrusHwInterface::OrthrusImuSubCallback, this, std::placeholders::_1));
    }

    void ImuInterfaces::OrthrusImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {

    }
}