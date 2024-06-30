#include "orthrus_controller/interfaces/JoyInterface.hpp"

namespace orthrus_controller
{
    void JoyInterface::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
    }

    void JoyInterface::JoyCallback(const xbox_interfaces::msg::XboxControl::SharedPtr msg)
    {
        // mtx_.lock();

        // 菜单键组合键用于不常用组合键
        if (msg->start == 1)
        {
            if (msg->x == 1)
            {
                orthrus_interfaces_->robot_cmd.if_enable_power = 1;
            }

            if (msg->y == 1)
            {
                orthrus_interfaces_->robot_cmd.if_enable_power = 0;
            }

            if (msg->b == 1)
            {
                orthrus_interfaces_->robot_cmd.if_enable_calibration = 1;
            }
            else
            {
                orthrus_interfaces_->robot_cmd.if_enable_calibration = 0;
            }

            if (msg->a == 1)
            {
                orthrus_interfaces_->robot_cmd.if_enable_calibration_encoder = 1;
            }
            else
            {
                orthrus_interfaces_->robot_cmd.if_enable_calibration_encoder = 0;
            }

            if (msg->lb == 1)
            {
                orthrus_interfaces_->robot_target.if_enable = 1;
            }

            if (msg->rb == 1)
            {
                orthrus_interfaces_->robot_target.if_enable = 0;
            }
        }
        else
        {

        }
    }

    std::stringstream JoyInterface::GetJoyTarget()
    {
        std::stringstream ss;
        ss << "JoyInterface: " << orthrus_interfaces_->robot_target.gait_num << " " << std::endl;
        return ss;
    }
}