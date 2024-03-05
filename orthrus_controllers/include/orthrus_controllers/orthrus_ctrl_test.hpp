#pragma once

#include <rclcpp/rclcpp.hpp>

#include <orthrus_interfaces/msg/ctrl_cmd.hpp>
#include <orthrus_controllers/orthrus_param_def.hpp>
namespace othrus_ctrl
{
    class PositonCtrl
    {
    public:
        orthrus_interfaces::msg::CtrlCmd StandUp();

    private:
        orthrus_interfaces::msg::CtrlCmd ctrl_cmd_msg_;
    };
}
