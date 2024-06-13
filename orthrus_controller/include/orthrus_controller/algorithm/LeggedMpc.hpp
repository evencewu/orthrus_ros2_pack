#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"

#include "orthrus_controller/interfaces/OrthrusParma.hpp"
#include "orthrus_controller/interfaces/PinocchioInterface.hpp"

#include <iostream>

namespace orthrus_controller
{
    class LeggedMpc
    {
    public:
        template <typename NodeType>
        LeggedMpc(std::shared_ptr<NodeType> node)
        {
            node_ = node;
        }

        void Init(std::shared_ptr<JointState> joint_ptr,
                  std::shared_ptr<OdomState> odom_ptr,
                  std::shared_ptr<std::vector<TouchState>> touch_ptr,
                  std::shared_ptr<PinocchioInterface> pinocchio_ptr);

        void Update(rclcpp::Time time, rclcpp::Duration duration);

        std::stringstream Logger();
        
        Eigen::VectorXd Foot2JointForce();

        Eigen::Matrix3d VectorToSkewSymmetricMatrix(const Eigen::Vector3d &v);
        Eigen::Matrix3d VectorToDiagonalMatrix(const Eigen::Vector3d &v);

    private:
        std::shared_ptr<OdomState> odom_state_;
        std::shared_ptr<JointState> joint_state_;
        std::shared_ptr<std::vector<TouchState>> touch_state_;

        std::shared_ptr<PinocchioInterface> pinocchio_interface_; // Pinocchio接口

        Eigen::VectorXd torq_;

        std::vector<std::string> foot_name_ = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};

        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        Eigen::VectorXd Body2FootForce(Eigen::VectorXd body_force);
        Eigen::MatrixXd body2footforce_mat_ = Eigen::MatrixXd::Random(6, 12);
        Eigen::MatrixXd body2footforce_mat_plus_= Eigen::MatrixXd::Random(6, 12);

        // 最小二乘伪逆
        Eigen::MatrixXd GetMinimumTworamTMat(Eigen::MatrixXd input);

        Eigen::MatrixXd GetTMat(Eigen::MatrixXd input);
    };
}