#include "orthrus_gazebo/orthrus_gazebo_interface.hpp"

namespace orthrus_gazebo
{
    orthrusGazeboNode::orthrusGazeboNode() : Node("orthrus_gazebo")
    {
        Init();
        
        joint_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);

        ctrl_cmd_sub_ = this->create_subscription<orthrus_interfaces::msg::CtrlCmd>("/orthrus_cmd", 10,
                                                                                    std::bind(&orthrusGazeboNode::CtrlCmdCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                                                                                   std::bind(&orthrusGazeboNode::JointStateCallback, this, std::placeholders::_1));
    }

    void orthrusGazeboNode::Init()
    {
        for (int i = 0; i < 12; i++)
        {
            JointParam_[i].name = joint_name_[i];
        }
    }

    void orthrusGazeboNode::CtrlCmdCallback(const orthrus_interfaces::msg::CtrlCmd::SharedPtr msg)
    {
        for (int i = 0; i < 12; i++)
        {
            hybrid_output_[i] = msg->motor_cmd[i].k_p * (msg->motor_cmd[i].target_p - JointParam_[i].position) + msg->motor_cmd[i].k_d * (msg->motor_cmd[i].target_d - JointParam_[i].velocity) + msg->motor_cmd[i].torqe;
        }

        joint_torque_msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        joint_torque_msg_.layout.dim[0].size = 12;

        joint_torque_msg_.data = hybrid_output_;

        joint_torque_pub_->publish(joint_torque_msg_);
    }

    void orthrusGazeboNode::JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                if (msg->name[i] == JointParam_[j].name)
                {
                    JointParam_[j].position = msg->position[i];
                    JointParam_[j].velocity = msg->velocity[i];
                    JointParam_[j].effort = msg->effort[i];
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_gazebo::orthrusGazeboNode>());
    rclcpp::shutdown();
    return 0;
}