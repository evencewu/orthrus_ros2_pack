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
            orthrus_interfaces_->robot_target.gait_num = 0;
        }

        if (msg->x == 1)
        {
            orthrus_interfaces_->robot_target.gait_num = 1;
        }

        // mtx_.unlock();
    }

    std::stringstream JoyInterface::GetJoyTarget()
    {
        std::stringstream ss;
        ss << "JoyInterface: " << orthrus_interfaces_->robot_target.gait_num << " " << std::endl;
        return ss;
    }
}