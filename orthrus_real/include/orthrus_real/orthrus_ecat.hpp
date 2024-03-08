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
    void init();
    void main_loop();

    void usleep(int usec);
    void safe_stop();
    void analyze_all();

    EcatBase Ethercat = EcatBase(1);

    // leg
    std::array<orthrus_real::Leg, 4> leg = {
        orthrus_real::Leg(CAN2, IMU1, USART1),
        orthrus_real::Leg(CAN2, IMU2, USART2),
        orthrus_real::Leg(CAN2, IMU3, USART3),
        orthrus_real::Leg(CAN2, IMU4, USART6)};

    // body imu
    Imu body_imu = Imu(CAN2, IMU5);

    //ros2 timer
    rclcpp::TimerBase::SharedPtr timer_;
  };
}