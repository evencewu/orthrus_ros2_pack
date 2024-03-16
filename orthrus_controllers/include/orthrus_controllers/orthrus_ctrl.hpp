#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_control.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_state.hpp>


#include <Eigen/Eigen>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "orthrus_controllers/orthrus_wbc.hpp"
#include "orthrus_controllers/orthrus_type_def.hpp"

//#define JOINT_INFO_LOG 

namespace orthrus_ctrl
{
    class orthrusCtrlNode : public rclcpp::Node
    {
    public:
        orthrusCtrlNode();
        //~orthrusCtrlNode() override;
    private:
        void init();

        void main_loop();

        void InitRobotParam();   // Init robot param of pinocchio
        void UpdateRobotParma(); //

        void SolveLegKinematics();
        void ResolveLegKinematics();
        void SolveLegDynamics();

        OrthrusParam OrthrusParam_;

        OrthrusWbc PositonCtrl_;

        std::vector<std::string> joint_name_{"hip_RF_joint", "leg1_RF_joint", "leg2_RF_joint", "hip_LF_joint", "leg1_LF_joint", "leg2_LF_joint", "hip_RB_joint", "leg1_RB_joint", "leg2_RB_joint", "hip_LB_joint", "leg1_LB_joint", "leg2_LB_joint"};

        // timer
        rclcpp::TimerBase::SharedPtr timer_;

        // orthrus_joint_control_pub_
        rclcpp::Publisher<orthrus_interfaces::msg::OrthrusJointControl>::SharedPtr orthrus_joint_control_pub_;
        orthrus_interfaces::msg::OrthrusJointControl orthrus_joint_control_msg_;

        // orthrus_joint_state_sub_
        rclcpp::Subscription<orthrus_interfaces::msg::OrthrusJointState>::SharedPtr orthrus_joint_state_sub_;
        orthrus_interfaces::msg::OrthrusJointState orthrus_joint_state_msg_;

        void OrthrusJointStateSubCallback(const orthrus_interfaces::msg::OrthrusJointState::SharedPtr msg);

        // orthrus_viewer_joint_state_pub_
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr orthrus_viewer_joint_state_pub_;
        sensor_msgs::msg::JointState orthrus_viewer_joint_state_msg_;

        // pinochio
        pinocchio::Model orthrus_model_;
        pinocchio::Data orthrus_data_;

        Eigen::VectorXd q_;
        Eigen::VectorXd v_;

        //sensor_msgs/msg/JointState

    };
}