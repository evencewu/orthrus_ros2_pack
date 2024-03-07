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
        InitDefKinematicsParma();

        for (int i = 0; i < 12; i++)
        {
            OrthrusParam_.joint[i].name = joint_name_[i];
        }
    }

    void orthrusCtrlNode::main_loop()
    {
        UpdateKinematicsParma();                      
        ctrl_cmd_msg_ = PositonCtrl_.StandUp();
        ctrl_cmd_pub_->publish(ctrl_cmd_msg_);
    }

    void orthrusCtrlNode::JointStateSubCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                if (msg->name[i] == OrthrusParam_.joint[j].name)
                {
                    OrthrusParam_.joint[j].position = msg->position[i];
                    OrthrusParam_.joint[j].velocity = msg->velocity[i];
                    OrthrusParam_.joint[j].effort = msg->effort[i];
                }
            }
        }
    }

    void orthrusCtrlNode::InitDefKinematicsParma()
    {
        RCLCPP_INFO(this->get_logger(), "run orthrus parma define\n");

        RCLCPP_INFO(this->get_logger(), "pinocchio urdf model loading\n");
        const std::string urdf_filename = std::string("/home/evence/code_file/ros2_ws/orthrus/src/orthrus_ros2_pack/orthrus_sim/orthrus_gazebo/models/orthrus/urdf/orthrus.urdf");

        pinocchio::urdf::buildModel(urdf_filename, orthrus_model_);
        RCLCPP_INFO(this->get_logger(), "over! model name: %s \n", orthrus_model_.name.c_str());

        // Create data required by the algorithms
        orthrus_data_ = pinocchio::Data(orthrus_model_);

        // Sample a random configuration
        OrthrusParam_.dynamic.joint_pos = randomConfiguration(orthrus_model_);

        // Perform the forward kinematics over the kinematic tree
        forwardKinematics(orthrus_model_, orthrus_data_, OrthrusParam_.dynamic.joint_pos);

        // Print out the placement of each joint of the kinematic tree
    }

    void orthrusCtrlNode::UpdateKinematicsParma()
    {
        for (int joint_id = 0; joint_id < 12; joint_id++)
        {
            OrthrusParam_.dynamic.joint_pos(joint_id) = OrthrusParam_.joint[joint_id].position;
        }

        forwardKinematics(orthrus_model_, orthrus_data_, OrthrusParam_.dynamic.joint_pos);

        RCLCPP_INFO(this->get_logger(), "|   Joint Name   |         Link Position         | Joint Angle |");
        for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)orthrus_model_.njoints; ++joint_id)
        {
            RCLCPP_INFO(this->get_logger(), "| %-14s | %9lf %9lf %9lf | %11lf |",
                        orthrus_model_.names[joint_id].c_str(),
                        orthrus_data_.oMi[joint_id].translation()(0),
                        orthrus_data_.oMi[joint_id].translation()(1),
                        orthrus_data_.oMi[joint_id].translation()(2),
                        (joint_id > 0) ? OrthrusParam_.dynamic.joint_pos(joint_id - 1) : 0);
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_ctrl::orthrusCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
