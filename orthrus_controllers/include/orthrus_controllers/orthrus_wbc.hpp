#pragma once

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Eigen>

#include <orthrus_interfaces/msg/orthrus_joint_control.hpp>
#include "orthrus_controllers/orthrus_type_def.hpp"

namespace orthrus_ctrl
{
    class OrthrusWbc
    {
    public:
        OrthrusWbc();

        //void BreakDownMotorForce(std::vector<double> force);
        
        orthrus_interfaces::msg::OrthrusJointControl StandUp();

    private:
        orthrus_interfaces::msg::OrthrusJointControl orthrus_joint_control_msg_;
    };
}