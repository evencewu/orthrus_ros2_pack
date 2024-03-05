#pragma once

#include <rclcpp/rclcpp.hpp>
//#include <Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

#include <orthrus_interfaces/msg/ctrl_cmd.hpp>


namespace othrus_ctrl
{
    const double PI = 3.1415926;

    class othrus_parma_def
    {
    public:
        othrus_parma_def();
    };
}