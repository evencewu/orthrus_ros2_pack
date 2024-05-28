#include "orthrus_controllers/hardware/ImuInterfaces.hpp"

namespace orthrus_control
{
    ImuInterfaces::ImuInterfaces(const rclcpp::Node::SharedPtr &node) : node_(node)
    {
        orthrus_imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("/orthrus_interface/imu", 10, std::bind(&ImuInterfaces::OrthrusImuSubCallback, this, std::placeholders::_1));
    }

    void ImuInterfaces::OrthrusImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        orientation_[0] = msg->orientation.x;
        orientation_[1] = msg->orientation.y;
        orientation_[2] = msg->orientation.z;
        orientation_[3] = msg->orientation.w;

        angular_velocity_[0] = msg->angular_velocity.x;
        angular_velocity_[1] = msg->angular_velocity.y;
        angular_velocity_[2] = msg->angular_velocity.z;

        linear_acceleration_[0] = msg->linear_acceleration.x;
        linear_acceleration_[1] = msg->linear_acceleration.y;
        linear_acceleration_[2] = msg->linear_acceleration.z;
    }

    Eigen::Vector4d ImuInterfaces::getOrientation()
    {
        return orientation_;
    }

    Eigen::Vector3d ImuInterfaces::getAngularVelocity()
    {
        return angular_velocity_;
    }

    Eigen::Vector3d ImuInterfaces::getLinearAcceleration()
    {
        return linear_acceleration_;
    }
}