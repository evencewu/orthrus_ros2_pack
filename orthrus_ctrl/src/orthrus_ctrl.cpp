#include "orthrus_ctrl/orthrus_ctrl.hpp"

//
namespace othrus_ctrl
{
    OthrusCtrlNode::OthrusCtrlNode() : Node("othrus_ctrl")
    {
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&OthrusCtrlNode::main_loop, this));
    }

    void OthrusCtrlNode::main_loop()
    {
        // 创建一个joint_state消息
        sensor_msgs::msg::JointState joint_state;
        joint_state.name = {"joint1", "joint2", "joint3"};
        joint_state.position = {0.1, 0.2, 0.3};
        joint_state.velocity = {0.4, 0.5, 0.6};
        joint_state.effort = {0.7, 0.8, 0.9};

        // 发布消息
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", joint_state.name[0].c_str());
        joint_state_pub_->publish(joint_state);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<othrus_ctrl::OthrusCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
