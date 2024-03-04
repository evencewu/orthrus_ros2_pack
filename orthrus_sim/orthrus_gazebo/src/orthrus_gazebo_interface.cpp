#include "orthrus_gazebo/orthrus_gazebo_interface.hpp"

namespace othrus_gazebo
{
    OthrusGazeboNode::OthrusGazeboNode() : Node("othrus_gazebo")
    {
        joint_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);

        ctrl_cmd_pub_ = this->create_subscription<orthrus_interfaces::msg::CtrlCmd>("/orthrus_cmd", 10,
            std::bind(&OthrusGazeboNode::CtrlCmdCallback, this, std::placeholders::_1));
    }

    void OthrusGazeboNode::CtrlCmdCallback(const orthrus_interfaces::msg::CtrlCmd::SharedPtr msg)
    {
        for(int i = 0;i<12;i++)
        {
            hybrid_[i] = msg->motor_cmd[i].torqe;
        }


        RCLCPP_INFO(this->get_logger(), "publish\n");
        joint_torque_msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        joint_torque_msg_.layout.dim[0].size = 12;

        std::vector<double> output_data {-30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30, -30};
        joint_torque_msg_.data = data;

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