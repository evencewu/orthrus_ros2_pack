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

        void OdomFilterInit(int filter_type);
        void OdomFilterUpdate();
        std::stringstream OdomFilterLog();

        Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond quat);

    private:
        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        int filter_type_ = 0;

        // EKF
        struct FilterEKF
        {
            Eigen::VectorXd x = Eigen::VectorXd::Zero(5);      // x_x x_y v_x v_y Status vector
            Eigen::VectorXd x_last = Eigen::VectorXd::Zero(5); // x_x x_y v_x v_y Status vector

            Eigen::VectorXd u = Eigen::VectorXd::Zero(3); // a_x a_y w_z

            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(5, 5); // Status transfer matrix
            Eigen::MatrixXd G = Eigen::MatrixXd::Zero(5, 3); // Status transfer matrix
        } ekf;

        struct FilterKF
        {
            // 先验估计
            Eigen::VectorXd x_priori = Eigen::VectorXd::Zero(6);

            // 后验估计
            Eigen::VectorXd x = Eigen::VectorXd::Zero(6);      // x_x x_y v_x v_y a_x a_yStatus vector
            Eigen::VectorXd x_last = Eigen::VectorXd::Zero(6); // x_x x_y v_x v_y a_x a_yStatus vector

            Eigen::VectorXd u = Eigen::VectorXd::Zero(2); // a_x a_y

            Eigen::VectorXd z = Eigen::VectorXd::Zero(2);

            Eigen::MatrixXd F = Eigen::MatrixXd::Zero(6, 6); // Status transfer matrix
            Eigen::MatrixXd G = Eigen::MatrixXd::Zero(6, 2); // Status transfer matrix

            Eigen::MatrixXd P = Eigen::MatrixXd::Zero(6, 6);
            Eigen::MatrixXd P_last = Eigen::MatrixXd::Zero(6, 6);
            Eigen::MatrixXd P_priori = Eigen::MatrixXd::Zero(6, 6);

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 6);

            Eigen::MatrixXd K = Eigen::MatrixXd::Zero(6, 2);

            Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
            Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);

            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);

            double d_a_x = 0.01; // 加速度计方差
            double d_a_y = 0.1;
        } kf;

        static const int EKF = 0;
        static const int KF = 1;
    };
}