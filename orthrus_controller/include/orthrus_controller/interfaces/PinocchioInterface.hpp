#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"

#include <iostream>

#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/geometry.hpp"

namespace orthrus_controller
{
    class PinocchioInterface
    {
    public:
        template <typename NodeType>
        PinocchioInterface(std::shared_ptr<NodeType> node)
        {
            node_ = node;
        }

        void Init();
        void Update();

        std::stringstream Logger();

        pinocchio::Model model_;

        pinocchio::Data data_;
        pinocchio::GeometryData collision_data_;
        pinocchio::GeometryData visual_data_;

        pinocchio::GeometryModel collision_model_;
        pinocchio::GeometryModel visual_model_;

        Eigen::VectorXd joint_;

        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;

    private:
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        std::string model_path_ = "/home/orthrus/orthrus/src/orthrus_ros2_pack/orthrus_interfaces/models/orthrus";
        std::string mesh_dir_ = model_path_ + "/meshes";
        std::string urdf_filename_ = model_path_ + "/urdf/orthrus.urdf";
    };
}