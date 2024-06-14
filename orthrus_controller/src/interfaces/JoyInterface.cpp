#include "orthrus_controller/interfaces/JoyInterface.hpp"

namespace orthrus_controller
{
    void JoyInterface::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;

        xbox_sub_ = node->template create_subscription<xbox_interfaces::msg::XboxControl>(
            "/xbox", 2, std::bind(&JoyInterface::JoyCallback, this, std::placeholders::_1));
    }

    void JoyInterface::JoyCallback(const xbox_interfaces::msg::XboxControl::SharedPtr msg)
    {

        // target_ptr_->target_velocity = msg.
    }
}