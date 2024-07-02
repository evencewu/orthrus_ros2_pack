#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "controller_interface/controller_interface.hpp"

#include "orthrus_controller/interfaces/OrthrusInterfaces.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orthrus_controller
{
    class LeggedOdom
    {
    public:
        template <typename NodeType>
        LeggedOdom(std::shared_ptr<NodeType> node)
        {
            node_ = node;
            RCLCPP_INFO(node->get_logger(), "Legged odom active.");
        }

        void Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr);
        void Update(rclcpp::Time time, rclcpp::Duration duration);
        void Calibration(rclcpp::Time time, rclcpp::Duration duration);

        Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond quat);
    private:

        void VelocityPredict(rclcpp::Time time, rclcpp::Duration duration);

        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        //EKF
        struct EKF
        {
            Eigen::VectorXd x(5); //x_x x_y v_x v_y w_z Status vector

            Eigen::VectorXd u(3); //a_x a_y w_z

            Eigen::MatrixXd F = MatrixXd::Zero(5, 5); // Status transfer matrix
            Eigen::MatrixXd G = MatrixXd::Zero(5, 3); // Status transfer matrix

            std::vector<double> x;
            std::vector<double> x;
        }ekf;
        

    };
}