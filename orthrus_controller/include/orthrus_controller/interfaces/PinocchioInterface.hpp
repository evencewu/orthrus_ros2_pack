#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "controller_interface/controller_interface.hpp"

#include "orthrus_controller/interfaces/OrthrusParma.hpp"

#include <iostream>

#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

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

        void Init(std::shared_ptr<JointState> joint_ptr,
                  std::shared_ptr<std::vector<TouchState>> touch_ptr);
        void Update(rclcpp::Time time);

        void FootPositionCalculation();
        void GravityCompensation();

        Eigen::VectorXd LegGravityCompensation();

        Eigen::MatrixXd GetJacobianMatrix(std::string frame_name);

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
        std::shared_ptr<std::vector<TouchState>> touch_state_;

        std::vector<std::string> foot_name_ = {"LF_FOOT","LH_FOOT","RF_FOOT","RH_FOOT"};
        
        std::variant<rclcpp::Node::SharedPtr, rclcpp_lifecycle::LifecycleNode::SharedPtr> node_;

        std::string model_path_ = "/home/orthrus/orthrus/src/orthrus_ros2_pack/orthrus_interfaces/models/orthrus";
        std::string mesh_dir_ = model_path_ + "/meshes";
        std::string urdf_filename_ = model_path_ + "/urdf/orthrus.urdf";
    };
}