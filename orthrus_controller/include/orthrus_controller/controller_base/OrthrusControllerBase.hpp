#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <mutex>

#include "controller_interface/controller_interface.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace orthrus_controller
{
    class OrthrusControllerBase : public controller_interface::ControllerInterface
    {

    public:
        //virtual
        DIFF_TEST_CONTROLLER_PUBLIC
        OrthrusControllerBase();

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time,
            const rclcpp::Duration &period) override;

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
            override;

        DIFF_TEST_CONTROLLER_PUBLIC
        
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
            override;

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
            override;

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
            override;

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state)
            override;

        DIFF_TEST_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
            override;

    protected:
        rclcpp::Time last_time_;
        rclcpp::Time now_time_;

        struct JointHandle
        {
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_position;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_velocity;
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_effort;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> cmd_effort;
        };

        std::vector<JointHandle> joint_handles_;

        controller_interface::CallbackReturn configure_joint(
            const std::vector<std::string> &joint_names,
            std::vector<JointHandle> &registered_handles);

        struct ImuHandle
        {
            std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> angular_velocity;
            std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> linear_acceleration;
            std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> orientation;
        };

        std::vector<ImuHandle> imu_handles_;

        controller_interface::CallbackReturn configure_imu(
            const std::vector<std::string> &imu_data_types,
            const std::vector<std::string> &imu_names,
            std::vector<ImuHandle> &registered_handles);

        struct LegImuHandle
        {
            std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> orientation;
        };

        std::vector<LegImuHandle> leg_imu_handles_;

        controller_interface::CallbackReturn configure_leg_imu(
            const std::vector<std::string> &flag_data_types,
            const std::vector<std::string> &flag_names,
            std::vector<LegImuHandle> &registered_handles);

        struct FlagHandle
        {
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> enable_power;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> calibration_position;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> calibration_encoder;
        };

        std::vector<FlagHandle> flag_handles_;

        controller_interface::CallbackReturn configure_flag(
            const std::vector<std::string> &flag_data_types,
            const std::vector<std::string> &flag_names,
            std::vector<FlagHandle> &registered_handles);
        // SafeCode
        std::shared_ptr<SafeCode> safe_code_;

        // parma
        std::shared_ptr<orthrus_controller::ParamListener> param_listener_;
        orthrus_controller::Params params_;
    };

} // namespace diff_test_controller