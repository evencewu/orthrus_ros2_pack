#pragma once

#include "rclcpp/rclcpp.hpp"
#include <orthrus_interfaces/msg/orthrus_joint_control.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_state.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace orthrus_control
{
    class OrthrusInterface
    {
    public:
        OrthrusInterface(const rclcpp::Node::SharedPtr &node);
        ~OrthrusInterface();

        void Init();

    private:

        rclcpp::Node::SharedPtr node_;
        
        std::vector<std::string> joint_name_{"hip_RF_joint", "leg1_RF_joint", "leg2_RF_joint", "hip_LF_joint", "leg1_LF_joint", "leg2_LF_joint", "hip_RB_joint", "leg1_RB_joint", "leg2_RB_joint", "hip_LB_joint", "leg1_LB_joint", "leg2_LB_joint"};

        //接收motor消息并发送关节信息
        rclcpp::Subscription<orthrus_interfaces::msg::OrthrusJointState>::SharedPtr orthrus_joint_state_sub_;
        orthrus_interfaces::msg::OrthrusJointState orthrus_joint_state_msg_;

        void OrthrusJointStateSubCallback(const orthrus_interfaces::msg::OrthrusJointState::SharedPtr msg);

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr orthrus_viewer_joint_state_pub_;
        sensor_msgs::msg::JointState orthrus_viewer_joint_state_msg_;

        //接收imu消息并发送odom信息
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orthrus_imu_sub_;
        sensor_msgs::msg::Imu orthrus_imu_msg_;

        void OrthrusImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        // orthrus_viewer_Horizontal_pub_
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr orthrus_viewer_horizontal_pub_;
        tf2_msgs::msg::TFMessage orthrus_viewer_horizontal_msg_;
    };
}
