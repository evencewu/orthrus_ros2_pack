#include "orthrus_controllers/estimate/StateEstimateBase.hpp"

namespace orthrus_control
{
    StateEstimateBase::StateEstimateBase(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info, const ocs2::PinocchioEndEffectorKinematics &eeKinematics, const rclcpp::Node::SharedPtr &node)
        : node_(node),
          pinocchioInterface_(std::move(pinocchioInterface)),
          info_(std::move(info)),
          eeKinematics_(eeKinematics.clone()),
          rbdState_(ocs2::vector_t::Zero(2 * info_.generalizedCoordinatesNum))
    {
        orthrus_joint_control_pub_ = node_->create_publisher<orthrus_interfaces::msg::OrthrusJointControl>("/orthrus_interface/joint_control", 10);
    }

    void StateEstimateBase::UpdateJointStates(const ocs2::vector_t &jointPos, const ocs2::vector_t &jointVel)
    {
        rbdState_.segment(6, info_.actuatedDofNum) = jointPos;
        rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = jointVel;
    }
}