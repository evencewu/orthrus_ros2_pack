#pragma once

#include <rclcpp/rclcpp.hpp>

#include <orthrus_interfaces/msg/ctrl_cmd.hpp>

#define PI 3.1415926
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
