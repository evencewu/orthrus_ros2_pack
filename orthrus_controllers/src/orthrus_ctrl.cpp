#include "orthrus_controllers/orthrus_ctrl.hpp"

namespace othrus_ctrl
{
    void JointHybrid::Compute(double target_position, double target_velocity, double effort)
    {
        *output_ = k_p_ * (*position_ - target_position) - k_d_ * (*velocity_ - target_velocity) + effort;
    }

    void JointHybrid::SetPd(double k_p, double k_d)
    {
        k_p_ = k_p;
        k_d_ = k_d;
    }

    void JointHybrid::SetParam(double *output, double *position, double *velocity)
    {
        output_ = output;
        position_ = position;
        velocity_ = velocity;
    }

}

namespace othrus_ctrl
{
    OthrusCtrlNode::OthrusCtrlNode() : Node("othrus_ctrl")
    {
        init();

        joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&OthrusCtrlNode::JointStateSubCallback, this, std::placeholders::_1));

        // joint_state_sub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&OthrusCtrlNode::main_loop, this));
    }

    void OthrusCtrlNode::init()
    {
        interfaces_mode_ = ISSACSIM;

        for (int i = 0; i < kJointNum; i++)
        {
            JointHybrid_[i].SetParam(&JointCmd_[i].effort, &JointStates_[i].position, &JointStates_[i].velocity);
            JointHybrid_[i].SetPd(10, -1);
        }
    }

    void OthrusCtrlNode::JointStateSubCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < kJointNum; i++)
        {
            for (int j = 0; j < kJointNum; j++)
            {
                if (msg->name[i].c_str() == joint_name_[j])
                {
                    JointStates_[j].position = msg->position[i];
                    JointStates_[j].velocity = msg->velocity[i];
                    JointStates_[j].effort = msg->effort[i];
                    RCLCPP_INFO(this->get_logger(), "%s %lf %lf %lf\n", msg->name[i].c_str(), JointStates_[j].position, JointStates_[j].velocity, JointStates_[j].effort);
                }
            }
        }
    }

    void OthrusCtrlNode::main_loop()
    {
        // set null
        joint_state_msg_.name = joint_name_;
        joint_state_msg_.position.resize(0);
        joint_state_msg_.velocity.resize(0);
        joint_state_msg_.effort.resize(0);

        JointHybrid_[0].Compute(0, 0, 0);
        JointHybrid_[1].Compute(-PI / 3, 0, 0);
        JointHybrid_[2].Compute(PI / 3, 0, 0);

        JointHybrid_[3].Compute(0, 0, 0);
        JointHybrid_[4].Compute(-PI / 3, 0, 0);
        JointHybrid_[5].Compute(PI / 3, 0, 0);

        JointHybrid_[6].Compute(0, 0, 0);
        JointHybrid_[7].Compute(PI / 3, 0, 0);
        JointHybrid_[8].Compute(-PI / 3, 0, 0);

        JointHybrid_[9].Compute(0, 0, 0);
        JointHybrid_[10].Compute(PI / 3, 0, 0);
        JointHybrid_[11].Compute(-PI / 3, 0, 0);

        if (interfaces_mode_ == ISSACSIM)
        {
            IssacSimTorqe();
            for (int i = 0; i < kJointNum; i++)
            {
                joint_state_msg_.position.insert(joint_state_msg_.position.end(), JointCmd_[i].position);
                joint_state_msg_.velocity.insert(joint_state_msg_.velocity.end(), JointCmd_[i].velocity);
                // RCLCPP_INFO(this->get_logger(), "%lf %lf\n", JointCmd_[i].position, JointCmd_[i].velocity);
            }
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
        // RCLCPP_INFO(this->get_logger(), "publish\n");
        joint_command_pub_->publish(joint_state_msg_);
    }

    void OthrusCtrlNode::IssacSimTorqe()
    {
        for (int i = 0; i < kJointNum; i++)
        {
            JointCmd_[i].position = JointStates_[i].position - JointCmd_[i].effort / (2 * ISSAC_K_P);
            JointCmd_[i].velocity = JointStates_[i].velocity - JointCmd_[i].effort / (2 * ISSAC_K_D);
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
