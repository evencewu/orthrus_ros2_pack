#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"


#include "orthrus_controller/interfaces/PinocchioInterfaces.hpp"

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

        void Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr,
                  std::shared_ptr<PinocchioInterfaces> pinocchio_ptr);

        void Update(rclcpp::Time time, rclcpp::Duration duration);

        std::stringstream Logger(int log_num);
        
        Eigen::VectorXd Foot2JointForce(); //足端力分配
        Eigen::Matrix3d VectorToSkewSymmetricMatrix(const Eigen::Vector3d &v);
        
        //Log

        static const int BODY2FOOTFORCE_LOG = 0;
        static const int JOINT_EFFOT_LOG = 1;
        static const int GRAVITY_EFFOT_LOG = 2;

    private:
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;
        std::shared_ptr<PinocchioInterfaces> pinocchio_interfaces_; // Pinocchio接口

        double dt_;
        //
        void GetBodyForcePD();
        Eigen::Vector3d position_last_;
        Eigen::Vector3d angular_last_;

        //
        Eigen::VectorXd torq_;

        std::vector<std::string> foot_name_ = {"LF_FOOT", "LH_FOOT", "RF_FOOT", "RH_FOOT"};

        Eigen::VectorXd Body2FootForce(Eigen::VectorXd body_force, std::vector<bool> gait_touch_sequence);
        Eigen::MatrixXd body2footforce_mat_ = Eigen::MatrixXd::Zero(6, 12);
        Eigen::MatrixXd body2footforce_mat_plus_= Eigen::MatrixXd::Zero(6, 12);

        // 最小二乘伪逆
        Eigen::MatrixXd GetMinimumTworamTMat(Eigen::MatrixXd input); //最小二乘伪逆
    };
}