#include "orthrus_controllers/orthrus_ctrl.hpp"

namespace othrus_ctrl
{
    OthrusCtrlNode::OthrusCtrlNode() : Node("othrus_ctrl")
    {
        init();

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&OthrusCtrlNode::JointStateSubCallback, this, std::placeholders::_1));

        ctrl_cmd_pub_ = this->create_publisher<orthrus_interfaces::msg::CtrlCmd>("/orthrus_cmd", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), std::bind(&OthrusCtrlNode::main_loop, this));
    }

    void OthrusCtrlNode::init()
    {
        for (int i = 0; i < 12; i++)
        {
            JointParam_[i].name = joint_name_[i];
        }
    }

    void OthrusCtrlNode::main_loop()
    {
        ctrl_cmd_msg_ = PositonCtrl_.StandUp();
        //num_++;
        //if (num_ <= 5000)
        //{
        //    ctrl_cmd_msg_.motor_cmd[0].k_p = 10;
        //    ctrl_cmd_msg_.motor_cmd[0].target_p = -PI / 2;
        //    ctrl_cmd_msg_.motor_cmd[0].k_d = 2;
        //    ctrl_cmd_msg_.motor_cmd[0].target_d = 0;
        //    ctrl_cmd_msg_.motor_cmd[0].torqe = 0;
        //}
        //else if (num_ > 5000 && num_ <= 10000)
        //{
        //    ctrl_cmd_msg_.motor_cmd[0].k_p = 10;
        //    ctrl_cmd_msg_.motor_cmd[0].target_p = PI / 2;
        //    ctrl_cmd_msg_.motor_cmd[0].k_d = 2;
        //    ctrl_cmd_msg_.motor_cmd[0].target_d = 0;
        //    ctrl_cmd_msg_.motor_cmd[0].torqe = 0;
        //}
        //else
        //{
        //    num_ = 0;
        //}
        ctrl_cmd_pub_->publish(ctrl_cmd_msg_);
    }

    void OthrusCtrlNode::JointStateSubCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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
    rclcpp::spin(std::make_shared<othrus_ctrl::OthrusCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
