#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

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

#include "orthrus_controller/visibility_control.h"

#include "orthrus_controller/visualization/OrthrusVisualization.hpp"

// auto-generated by generate_parameter_library
#include "orthrus_controller_parameters.hpp"

namespace orthrus_controller
{

  class OrthrusController : public controller_interface::ControllerInterface
  {

  public:
    DIFF_TEST_CONTROLLER_PUBLIC
    OrthrusController();

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

    // 可视化

    std::shared_ptr<OrthrusVisualization> visualization_;

    // 里程计
    // Odometry odometry_;
    // 发布里程计数据
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
        realtime_odometry_publisher_ = nullptr;

    // 发布坐标变化关系
    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_ =
        nullptr;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
        realtime_odometry_transform_publisher_ = nullptr;

    // Parameters from ROS for orthrus_controller
    std::shared_ptr<orthrus_controller::ParamListener> param_listener_;
    orthrus_controller::Params params_;

    // received command
    bool subscriber_is_active_ = false;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber_ = nullptr;
    realtime_tools::RealtimeBox<std::shared_ptr<geometry_msgs::msg::Twist>> received_velocity_msg_ptr_{nullptr};
    std::queue<geometry_msgs::msg::Twist> previous_commands_; // last two commands

    // x speed, positive means forward,unit m/s. 速度 X方向 前为正
    // z rotation speed, positive means counterclockwise,unit
    // rad/s.Yaw旋转角速度，逆时针为正 单位 rad/s
    double linear_command;
    double angular_command;

    bool is_halted = false;
    bool reset();
    void halt();
  };

} // namespace diff_test_controller
