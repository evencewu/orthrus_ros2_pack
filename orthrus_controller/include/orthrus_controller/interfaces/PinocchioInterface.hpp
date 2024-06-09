#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"

#include "orthrus_controller/interfaces/OrthrusParma.hpp"

#include <iostream>

#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <Eigen/Dense>

namespace orthrus_controller
{
    class PinocchioInterface : public std::enable_shared_from_this<PinocchioInterface>
    {
    public:
        template <typename NodeType>
        PinocchioInterface(std::shared_ptr<NodeType> node)
        {
            node_ = node;
        }

        void Init(std::shared_ptr<JointState> joint_state_ptr);
        void Update();
        std::stringstream LegPositionInterpolation();

        std::stringstream Logger();

        pinocchio::Model model_;

        pinocchio::Data data_;
        pinocchio::GeometryData collision_data_;
        pinocchio::GeometryData visual_data_;

        pinocchio::GeometryModel collision_model_;
        pinocchio::GeometryModel visual_model_;

        Eigen::VectorXd joint_;
    private:
        std::shared_ptr<JointState> joint_state_;

        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        std::string model_path_ = "/home/evence/code_file/ros2_ws/orthrus/src/orthrus_ros2_pack/orthrus_interfaces/models/orthrus";
        std::string mesh_dir_ = model_path_ + "/meshes";
        std::string urdf_filename_ = model_path_ + "/urdf/orthrus.urdf";
    };
}