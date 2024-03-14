#pragma once

#include <rclcpp/rclcpp.hpp>

#include <orthrus_interfaces/msg/ctrl_cmd.hpp>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace orthrus_ctrl
{
    const double PI = 3.1415926;

    struct JointParam
    {
        double position;
        double velocity;
        double effort;
    };

    struct Dynamic
    {
        Eigen::VectorXd joint_pos;
    };

    struct OrthrusParam
    {
        JointParam joint[12];
        Dynamic dynamic;
        
    };
}
