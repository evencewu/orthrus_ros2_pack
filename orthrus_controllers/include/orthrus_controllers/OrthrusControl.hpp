#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_control.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace orthrus_control
{
    class OrthrusControlNode : public rclcpp::Node
    {
    public:
        using CmdToTargetTrajectories = std::function<ocs2::TargetTrajectories(const ocs2::vector_t &cmd, const ocs2::SystemObservation &observation)>;

        OrthrusControlNode();

    private:
        void MainLoop();

        struct JointParam
        {
            double position;
            double velocity;
            double effort;
        };

        struct OrthrusParam
        {
            JointParam joint[12];
        }OrthrusParam_;

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

        // orthrus_imu_sub_
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orthrus_imu_sub_;
        sensor_msgs::msg::Imu orthrus_imu_msg_;

        void OrthrusImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        // orthrus_viewer_joint_state_pub_
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr orthrus_viewer_joint_state_pub_;
        sensor_msgs::msg::JointState orthrus_viewer_joint_state_msg_;

        // orthrus_viewer_Horizontal_pub_
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr orthrus_viewer_horizontal_pub_;
        tf2_msgs::msg::TFMessage orthrus_viewer_horizontal_msg_;

        CmdToTargetTrajectories goalToTargetTrajectories;
        CmdToTargetTrajectories cmdVelToTargetTrajectories;

        ocs2::SystemObservation latestObservation_;
    };
}