#include "orthrus_controllers/OrthrusControl.hpp"

namespace orthrus_control
{
    hardware_interface::CallbackReturn OrthrusController::on_init(const hardware_interface::HardwareInfo &info)
    {
        // RCLCPP_INFO(rclcpp::get_logger("orthrus_controller"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OrthrusController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        // RCLCPP_INFO(rclcpp::get_logger("orthrus_controller"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orthrus_control::OrthrusController, hardware_interface::SystemInterface);
