#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "orthrus_control/visibility_control.h"

#include "orthrus_control/ethercat/EcatBase.hpp"
#include "orthrus_control/ethercat/TypeDef.hpp"
#include "orthrus_control/assembly/Leg.hpp"
#include "orthrus_control/assembly/Imu.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace orthrus_control
{
    class OrthrusSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(OrthrusSystemHardware);

        ORTHRUS_TEST_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        ORTHRUS_TEST_CONTROL_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ORTHRUS_TEST_CONTROL_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ORTHRUS_TEST_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        ORTHRUS_TEST_CONTROL_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        ORTHRUS_TEST_CONTROL_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        ORTHRUS_TEST_CONTROL_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        double theta1 = 18.82;
        double theta2 = 14.6;
        // ethercat
        bool ethercat_prepare_flag_ = false;
        std::chrono::time_point<std::chrono::high_resolution_clock> time_last_ethercat_;
        std::chrono::time_point<std::chrono::high_resolution_clock> time_now_;
        double duration_;

        EcatBase Ethercat = EcatBase(2);

        // hardware interface

        std::shared_ptr<OrthrusControlVariable> variable_;

        Leg leg[4];
        Imu body_imu;
        double command_effort[12];

        int motorcan_send_flag_ = 0;

        //
        void UnifiedSensorData();
        
        void Log();
        void SafeStop();

        //Calibration
        void StartCalibrationEncoderPosition();
        void StopCalibrationEncoderPosition();

        void CalibrationPosition();
        double dealta_real_position_[4][3];


        //
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_effort_;

        std::vector<double> hw_commands_;

        std::vector<double> hw_sensor_states_;

        std::unordered_map<std::string, double> gpio_commands_;

        std::shared_ptr<rclcpp::Node> node_;
        // subscribe crawler feed back
        bool subscriber_is_active_ = false;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fb_subscriber_ = nullptr;
        realtime_tools::RealtimeBox<std::shared_ptr<std_msgs::msg::Float64MultiArray>> received_fb_msg_ptr_{nullptr};
        // publishe crawler conmand
        std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> realtime_cmd_publisher_ = nullptr;
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> cmd_publisher = nullptr;
    };
}
