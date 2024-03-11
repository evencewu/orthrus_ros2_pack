#pragma once

#include <rclcpp/rclcpp.hpp>

#include <orthrus_interfaces/msg/ctrl_cmd.hpp>

#include "orthrus_controllers/orthrus_type_def.hpp"

namespace orthrus_ctrl
{
    class OrthrusWbc
    {
    public:
        OrthrusWbc();

        TurnForce();
        
        orthrus_interfaces::msg::CtrlCmd StandUp();

    private:
        orthrus_interfaces::msg::CtrlCmd ctrl_cmd_msg_;
    };
}