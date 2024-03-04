#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <orthrus_interfaces/msg/ctrl_cmd.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace othrus_gazebo
{
    struct JointParam
    {
        std::string name;

        double position;
        double velocity;
        double effort;
    };

    class JointHybrid
    {
    public:
        void Compute(double target_position, double target_velocity, double effort);
        void SetPd(double k_p, double k_d);
        void SetParam(double *output, double *position, double *velocity);

        double *output_;
        double *position_;
        double *velocity_;

        double k_p_;
        double k_d_;
    };

    class OthrusGazeboNode : public rclcpp::Node
    {
    public:
        OthrusGazeboNode();

        void main_loop();

    private:
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_torque_pub_;
        std_msgs::msg::Float64MultiArray joint_torque_msg_;

        rclcpp::Subscription<orthrus_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_sub_;
        orthrus_interfaces::msg::CtrlCmd ctrl_cmd_msg_;

        void CtrlCmdCallback(const orthrus_interfaces::msg::CtrlCmd::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        sensor_msgs::msg::JointState joint_state_msg_;

        void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        std::vector<double> hybrid_output_ = std::vector<double>(12);

        JointParam joint_param_[12];

        std::vector<std::string> joint_name_{"hip_RF_joint", "leg1_RF_joint", "leg2_RF_joint", "hip_LF_joint", "leg1_LF_joint", "leg2_LF_joint", "hip_RB_joint", "leg1_RB_joint", "leg2_RB_joint", "hip_LB_joint", "leg1_LB_joint", "leg2_LB_joint"};
    };
}