#include "orthrus_ctrl/orthrus_ctrl.hpp"

namespace othrus_ctrl
{
    OthrusCtrlNode::OthrusCtrlNode() : Node("othrus_ctrl")
    {
        interfaces_mode_ = ISSACSIM;

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
        //joint_state_sub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&OthrusCtrlNode::main_loop, this));
    }

    void OthrusCtrlNode::main_loop()
    {
        // set null
        joint_state_.name = joint_name_;
        joint_state_.position.resize(0);
        joint_state_.velocity.resize(0);
        joint_state_.effort.resize(0);

        if (interfaces_mode_ == ISSACSIM)
        {
            for (int i = 0; i < 12; i++)
            {
               IssacSimJoint_[i].effort = 30;
            }
            IssacSimTorqe();
        }
        else if (interfaces_mode_ == REAL)
        {
            //
        }
        else
        {
            //
        }

        // 发布消息
        RCLCPP_INFO(this->get_logger(), "Publish");
        joint_state_pub_->publish(joint_state_);
    }

    void OthrusCtrlNode::IssacSimTorqe()
    {
        for (int i = 0; i < kJointNum; i++)
        {
            joint_state_.position.insert(joint_state_.position.begin(), IssacSimJoint_[i].position - IssacSimJoint_[i].effort / (2 * ISSAC_K_P));
            joint_state_.velocity.insert(joint_state_.velocity.begin(), IssacSimJoint_[i].velocity - IssacSimJoint_[i].effort / (2 * ISSAC_K_D));
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<othrus_ctrl::OthrusCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
