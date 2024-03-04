#pragma once

#include <rclcpp/rclcpp.hpp>

#include <orthrus_interfaces/msg/ctrl_cmd.hpp>

namespace othrus_ctrl
{
    struct JointParam
    {
        std::string name;

        double position;
        double velocity;
        double effort;
    };

    class OrthrusMotion
    {
    
    };

    class OrthrusDynamics
    {

    };    
}