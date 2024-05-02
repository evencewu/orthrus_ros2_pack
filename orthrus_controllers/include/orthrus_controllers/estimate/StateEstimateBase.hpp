#pragma once

#include "rclcpp/rclcpp.hpp"
#include <orthrus_interfaces/msg/orthrus_joint_control.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_state.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>

namespace orthrus_control
{
    class StateEstimateBase
    {
    public:
        StateEstimateBase(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info, const ocs2::PinocchioEndEffectorKinematics &eeKinematics, const rclcpp::Node::SharedPtr &node);

        void UpdateImu();
        void UpdateJointStates(const ocs2::vector_t &jointPos, const ocs2::vector_t &jointVel);
        void updateAngular(const ocs2::legged_robot::vector3_t &zyx, const ocs2::vector_t &angularVel);
        void updateLinear(const ocs2::vector_t &pos, const ocs2::vector_t &linearVel);
        void UpdateContact();

        ocs2::vector_t Update(const rclcpp::Time &time, const rclcpp::Duration &period);

    private:
        rclcpp::Node::SharedPtr node_;

        //joint pub 
        rclcpp::Publisher<orthrus_interfaces::msg::OrthrusJointControl>::SharedPtr orthrus_joint_control_pub_;
        orthrus_interfaces::msg::OrthrusJointControl orthrus_joint_control_msg_;

        ocs2::PinocchioInterface pinocchioInterface_;
        ocs2::CentroidalModelInfo info_;
        std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematics_;

        std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                             "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

        ocs2::vector_t rbdState_;
    };
}
