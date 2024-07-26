#pragma once

#include <orthrus_controller/controller_base/OrthrusControllerBase.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace orthrus_controller
{
    /*
    class OrthrusController : public OrthrusControllerBase
    {

    public:
        ORTHRUS_CONTROLLER_PUBLIC
        OrthrusController();

    private:
        
    };
    */

    class OrthrusController : public controller_interface::ControllerInterface
    {

    public:
        ORTHRUS_CONTROLLER_PUBLIC
        OrthrusController();

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
            override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
            override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
            override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
            override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state)
            override;

        ORTHRUS_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
            override;
    };
}