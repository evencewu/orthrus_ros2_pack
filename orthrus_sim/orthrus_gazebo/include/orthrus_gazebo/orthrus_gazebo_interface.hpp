#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <orthrus_interfaces/msg/ctrl_cmd.hpp>

namespace othrus_gazebo
{
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
        void CtrlCmdCallback(const orthrus_interfaces::msg::CtrlCmd::SharedPtr msg);

        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_torque_pub_;
        std_msgs::msg::Float64MultiArray joint_torque_msg_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<orthrus_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_pub_;
        orthrus_interfaces::msg::CtrlCmd ctrl_cmd_msg_;

        double hybrid_[12];
        
    };
}