#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <orthrus_interfaces/msg/ctrl_cmd.hpp>

#include "orthrus_controllers/orthrus_ctrl_base.hpp"
#include "orthrus_controllers/orthrus_ctrl_test.hpp"
#include "orthrus_controllers/orthrus_param_def.hpp"

#define PI 3.1415926

namespace othrus_ctrl
{
    class OthrusCtrlNode : public rclcpp::Node
    {
    public:
        OthrusCtrlNode();
        //~OthrusCtrlNode() override;

        void main_loop();
    private:
        void init();
        int num_ = 0;

        PositonCtrl PositonCtrl_;
        othrus_parma_def othrus_parma_;
        
        //timer
        rclcpp::TimerBase::SharedPtr timer_;
        
        //joint_state_sub_
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        sensor_msgs::msg::JointState joint_state_msg_;
        
        void JointStateSubCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        JointParam JointParam_[12];

        std::vector<std::string> joint_name_{"hip_RF_joint", "leg1_RF_joint", "leg2_RF_joint", "hip_LF_joint", "leg1_LF_joint", "leg2_LF_joint", "hip_RB_joint", "leg1_RB_joint", "leg2_RB_joint", "hip_LB_joint", "leg1_LB_joint", "leg2_LB_joint"};

        //ctrl_cmd_pub_
        rclcpp::Publisher<orthrus_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_pub_;
        orthrus_interfaces::msg::CtrlCmd ctrl_cmd_msg_;
    };
}