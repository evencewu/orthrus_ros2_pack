#include "orthrus_gazebo/orthrus_gazebo_interface.hpp"

namespace othrus_gazebo
{
    OthrusGazeboNode::OthrusGazeboNode() : Node("othrus_gazebo")
    {
        joint_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&OthrusGazeboNode::main_loop, this));
    }

    void OthrusGazeboNode::main_loop()
    {
        RCLCPP_INFO(this->get_logger(), "publish\n");
        joint_torque_msg_.data.push_back(0);

        joint_torque_msg_.data[0] = 5;
        joint_torque_pub_->publish(joint_torque_msg_);
    }
    
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<othrus_gazebo::OthrusGazeboNode>());
    rclcpp::shutdown();
    return 0;
}