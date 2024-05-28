#include "orthrus_controllers/hardware/HybridJointInterfaces.hpp"

namespace orthrus_control
{
    HybridJointInterfaces::HybridJointInterfaces(const rclcpp::Node::SharedPtr &node) :node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "HybridJointInterfaces init");
        orthrus_joint_state_sub_ = node_->create_subscription<orthrus_interfaces::msg::OrthrusJointState>("/orthrus_interface/joint_state", 10, std::bind(&HybridJointInterfaces::OrthrusJointStateSubCallback, this, std::placeholders::_1));
    }

    void HybridJointInterfaces::OrthrusJointStateSubCallback(const orthrus_interfaces::msg::OrthrusJointState::SharedPtr msg)
    {
        for (int i = 0; i < 12; i++)
        {
            position[i] = msg->motor[i].pos;
            velocity[i] = msg->motor[i].vec;
            effort[i] = msg->motor[i].torq;
        }
    }

    double HybridJointInterfaces::getPosition(int joint_num)
    {
        return position[joint_num];
    }

    double HybridJointInterfaces::getVelocity(int joint_num)
    {
        return velocity[joint_num];
    }
}