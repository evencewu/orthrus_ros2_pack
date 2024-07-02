#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_interface/controller_interface.hpp"

#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "orthrus_controller/interfaces/OrthrusInterfaces.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"

#include "orthrus_controller/interfaces/PinocchioInterfaces.hpp"

namespace orthrus_controller
{
    class OrthrusVisualization : public std::enable_shared_from_this<OrthrusVisualization>
    {
    public:
        template <typename NodeType>
        OrthrusVisualization(std::shared_ptr<NodeType> node, std::vector<std::string> joint_name) : joint_name_(joint_name)
        {
            node_ = node;

            joint_state_publisher_ = node->template create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            tf_publisher_ = node->template create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
            marker_publisher_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>("/orthrus/visualization_marker", 10);
            imu_publisher_ = node->template create_publisher<sensor_msgs::msg::Imu>("/orthrus/body_imu", 10);
        }

        void Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr);

        void Update(rclcpp::Time time);
        void ModelVisualization(rclcpp::Time time);

        void TfVisualization(rclcpp::Time time);
        void BodyTfVisualization(rclcpp::Time time);
        void FootTfVisualization(rclcpp::Time time);

        void MarkVisualization(rclcpp::Time time);
        void LegForceMarkVisualization(rclcpp::Time time);

        void ImuVisualization(rclcpp::Time time);

    private:
        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;

        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        std::vector<std::string> foot_names_ = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};
        std::vector<std::string> haa_names_ = {"LF_HAA", "LH_HAA", "RF_HAA", "RH_HAA"};

        std::vector<std::string> joint_name_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        sensor_msgs::msg::JointState joint_state_msg_;

        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
        tf2_msgs::msg::TFMessage tf_msg_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        visualization_msgs::msg::MarkerArray markerarray_msg_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        nav_msgs::msg::Odometry odom_msg_;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        sensor_msgs::msg::Imu imu_msg_;
    };
}