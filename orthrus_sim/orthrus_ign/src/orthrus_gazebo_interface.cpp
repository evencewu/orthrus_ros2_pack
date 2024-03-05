#include "orthrus_gazebo/orthrus_gazebo_interface.hpp"

namespace orthrus_gazebo
{
    orthrusGazeboNode::orthrusGazeboNode() : Node("orthrus_gazebo")
    {
        joint_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&orthrusGazeboNode::main_loop, this));
    }

    void orthrusGazeboNode::main_loop()
    {
        RCLCPP_INFO(this->get_logger(), "publish\n");
        joint_torque_msg_.data.push_back(0);

        joint_torque_msg_.data[0] = 30;
        joint_torque_pub_->publish(joint_torque_msg_);
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_gazebo::orthrusGazeboNode>());
    rclcpp::shutdown();
    return 0;
}