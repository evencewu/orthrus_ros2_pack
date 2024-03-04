#include "orthrus_gazebo/orthrus_gazebo_interface.hpp"

namespace othrus_gazebo
{
    OthrusGazeboNode::OthrusGazeboNode() : Node("othrus_gazebo")
    {
        for (int i = 0; i < 12; i++)
        {
            joint_param_[i].name = joint_name_[i];
        }

        joint_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);

        ctrl_cmd_sub_ = this->create_subscription<orthrus_interfaces::msg::CtrlCmd>("/orthrus_cmd", 10,
                                                                                    std::bind(&OthrusGazeboNode::CtrlCmdCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                                                                                   std::bind(&OthrusGazeboNode::JointStateCallback, this, std::placeholders::_1));
    }

    void OthrusGazeboNode::CtrlCmdCallback(const orthrus_interfaces::msg::CtrlCmd::SharedPtr msg)
    {
        for (int i = 0; i < 12; i++)
        {
            hybrid_output_[i] = msg->motor_cmd[i].k_p * (msg->motor_cmd[i].target_p - joint_param_[i].position) - msg->motor_cmd[i].k_d * (msg->motor_cmd[i].target_d - joint_param_[i].velocity) + msg->motor_cmd[i].torqe;
        }

        
        joint_torque_msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        joint_torque_msg_.layout.dim[0].size = 12;

        joint_torque_msg_.data = hybrid_output_;

        joint_torque_pub_->publish(joint_torque_msg_);
    }

    void OthrusGazeboNode::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                if (msg->name[i] == joint_param_[j].name)
                {
                    joint_param_[j].position = msg->position[i];
                    joint_param_[j].velocity = msg->velocity[i];
                    joint_param_[j].effort = msg->effort[i];
                    //RCLCPP_INFO(this->get_logger(), "find_name\n");
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<othrus_gazebo::OthrusGazeboNode>());
    rclcpp::shutdown();
    return 0;
}