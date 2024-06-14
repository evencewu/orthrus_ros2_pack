#include "orthrus_controller/interfaces/JoyInterface.hpp"

namespace orthrus_controller
{
    void JoyInterface::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
    }

    void JoyInterface::JoyCallback(const xbox_interfaces::msg::XboxControl::SharedPtr msg)
    {
        if(msg->y == 1)
        {
            orthrus_interfaces_->robot_target.gait_num = 0;
        }

        if(msg->x == 0)
        {
            orthrus_interfaces_->robot_target.gait_num = 1;
        }
        
        
        // target_ptr_->target_velocity = msg.
    }
}