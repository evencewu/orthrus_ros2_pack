#include "orthrus_controllers/orthrus_ctrl.hpp"

namespace orthrus_ctrl
{
    orthrusCtrlNode::orthrusCtrlNode() : Node("orthrus_ctrl")
    {
        init();

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&orthrusCtrlNode::JointStateSubCallback, this, std::placeholders::_1));

        ctrl_cmd_pub_ = this->create_publisher<orthrus_interfaces::msg::CtrlCmd>("/orthrus_cmd", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), std::bind(&orthrusCtrlNode::main_loop, this));
    }

    void orthrusCtrlNode::init()
    {
        orthrus_parma_def();

        for (int i = 0; i < 12; i++)
        {
            JointParam_[i].name = joint_name_[i];
        }
    }

    void orthrusCtrlNode::main_loop()
    {
        ctrl_cmd_msg_ = PositonCtrl_.StandUp();
        ctrl_cmd_pub_->publish(ctrl_cmd_msg_);
    }

    void orthrusCtrlNode::JointStateSubCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
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

    void orthrusCtrlNode::orthrus_parma_def()
    {
        RCLCPP_INFO(this->get_logger(), "run orthrus parma define\n");

        RCLCPP_INFO(this->get_logger(), "pinocchio urdf model loading\n");
        const std::string urdf_filename = std::string("/home/evence/code_file/ros2_ws/orthrus/src/orthrus_ros2_pack/orthrus_sim/orthrus_gazebo/models/orthrus/urdf/orthrus.urdf");

        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_filename, model);
        RCLCPP_INFO(this->get_logger(), "over! model name: %s \n", model.name.c_str());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_ctrl::orthrusCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
