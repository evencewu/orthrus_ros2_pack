#pragma once

// ROS
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "orthrus_real/ecat_base.hpp"
#include "orthrus_real/ecat_typedef.hpp"
#include "orthrus_real/orthrus_leg.hpp"
#include "orthrus_real/orthrus_imu.hpp"

namespace orthrus_real
{
  class OrthrusInterfacesNode : public rclcpp::Node
  {
  public:
    OrthrusInterfacesNode();
    ~OrthrusInterfacesNode() override;

  private:
    void Init();
    void MainLoop();
    void LedLoop();
    void LegLoop();

    void SafeStop();
    void AnalyzeAll();

    EcatBase Ethercat = EcatBase(1);

    // leg
    Leg leg[4];
    // body imu
    Imu body_imu;
    // ros2 timer
    rclcpp::TimerBase::SharedPtr timer_;

    int led_flag_ = 0;
    int motor_send_flag_ = 0;
    int motorcan_send_flag_ = 0;
    int motorcan_time_flag_ = 0;
  };
}