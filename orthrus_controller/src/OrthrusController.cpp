#include <orthrus_controller/OrthrusController.hpp>

#include "class_loader/register_macro.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

namespace orthrus_controller
{
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using lifecycle_msgs::msg::State;

    OrthrusController::OrthrusController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn OrthrusController::on_init()
    {
        auto logger = get_node()->get_logger();
        RCLCPP_INFO(get_node()->get_logger(), "Loading orthrus controller...");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration OrthrusController::command_interface_configuration() const
    {
        //return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    InterfaceConfiguration OrthrusController::state_interface_configuration() const
    {
        //return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::CallbackReturn OrthrusController::on_configure(
        const rclcpp_lifecycle::State &)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn OrthrusController::on_activate(
        const rclcpp_lifecycle::State &)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn OrthrusController::on_cleanup(
        const rclcpp_lifecycle::State &)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn OrthrusController::on_error(
        const rclcpp_lifecycle::State &)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn OrthrusController::on_deactivate(
        const rclcpp_lifecycle::State &)
    {

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn OrthrusController::on_shutdown(
        const rclcpp_lifecycle::State &)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type OrthrusController::update(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        return controller_interface::return_type::OK;
    }
}

CLASS_LOADER_REGISTER_CLASS(orthrus_controller::OrthrusController, controller_interface::ControllerInterface)