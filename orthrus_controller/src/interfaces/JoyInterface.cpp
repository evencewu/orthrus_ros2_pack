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

        if (msg->y == 1)
        {
            orthrus_interfaces_->robot_target.if_enable = 0;
        }

        if (msg->x == 1)
        {
            orthrus_interfaces_->robot_target.if_enable = 1;
        }

        if (msg->start == 1)
        {
            orthrus_interfaces_->robot_cmd.if_enable_power = 1;
        }

        if (msg->back == 1)
        {
            orthrus_interfaces_->robot_cmd.if_enable_power = 0;
        }
    }

    std::stringstream JoyInterface::GetJoyTarget()
    {
        std::stringstream ss;
        ss << "JoyInterface: " << orthrus_interfaces_->robot_target.gait_num << " " << std::endl;
        return ss;
    }
}