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

    controller_interface::InterfaceConfiguration OrthrusController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto &joint_name : params_.leg_joint_names)
        {
            conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
        }

        if (params_.sim_or_real == "real")
        {
            conf_names.push_back("flag/enable_power");
            conf_names.push_back("flag/calibration_position");
            conf_names.push_back("flag/calibration_encoder");
        }

        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration OrthrusController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto &joint_name : params_.leg_joint_names)
        {
            conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
            conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
            conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
        }

        if (params_.sim_or_real == "real")
        {
            for (const auto &leg_imu_name : params_.leg_imu_names)
            {
                conf_names.push_back(leg_imu_name + "/" + "orientation.w");
                conf_names.push_back(leg_imu_name + "/" + "orientation.x");
                conf_names.push_back(leg_imu_name + "/" + "orientation.y");
                conf_names.push_back(leg_imu_name + "/" + "orientation.z");
            }
        }

        conf_names.push_back("imu_sensor/angular_velocity.x");
        conf_names.push_back("imu_sensor/angular_velocity.y");
        conf_names.push_back("imu_sensor/angular_velocity.z");
        conf_names.push_back("imu_sensor/linear_acceleration.x");
        conf_names.push_back("imu_sensor/linear_acceleration.y");
        conf_names.push_back("imu_sensor/linear_acceleration.z");
        conf_names.push_back("imu_sensor/orientation.w");
        conf_names.push_back("imu_sensor/orientation.x");
        conf_names.push_back("imu_sensor/orientation.y");
        conf_names.push_back("imu_sensor/orientation.z");
        return {interface_configuration_type::INDIVIDUAL, conf_names};
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