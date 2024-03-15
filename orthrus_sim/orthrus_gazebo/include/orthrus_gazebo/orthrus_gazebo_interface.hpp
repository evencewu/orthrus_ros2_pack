#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_control.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace orthrus_gazebo
{
    struct JointParam
    {
        std::string name;

        double position;
        double velocity;
        double effort;
    };

    class orthrusGazeboNode : public rclcpp::Node
    {
    public:
        orthrusGazeboNode();

        void main_loop();

    private:
        void Init();

        std::vector<std::string> joint_name_{"hip_RF_joint", "leg1_RF_joint", "leg2_RF_joint", "hip_LF_joint", "leg1_LF_joint", "leg2_LF_joint", "hip_RB_joint", "leg1_RB_joint", "leg2_RB_joint", "hip_LB_joint", "leg1_LB_joint", "leg2_LB_joint"};

        // joint_torque_pub_
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_torque_pub_;
        std_msgs::msg::Float64MultiArray joint_torque_msg_;

        std::vector<double> hybrid_output_ = std::vector<double>(12);

        // joint_state_sub_
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

        void OrthrusGazeboJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        // orthrus_joint_state_pub_
        rclcpp::Publisher<orthrus_interfaces::msg::OrthrusJointState>::SharedPtr orthrus_joint_state_pub_;
        orthrus_interfaces::msg::OrthrusJointState orthrus_joint_state_msg_;
        
        // orthrus_joint_control_sub_
        rclcpp::Subscription<orthrus_interfaces::msg::OrthrusJointControl>::SharedPtr orthrus_joint_control_sub_;

        void OrthrusJointControlCallback(const orthrus_interfaces::msg::OrthrusJointControl::SharedPtr msg);

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr orthrus_imu_pub_;
        sensor_msgs::msg::Imu orthrus_imu_msg_;
        
        // orthrus_joint_control_sub_
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orthrus_gazebo_imu_sub_;
        sensor_msgs::msg::Imu orthrus_gazebo_imu_msg_;

        void OrthrusGazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        JointParam JointParam_[12];
    };
}