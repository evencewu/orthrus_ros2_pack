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

#include "orthrus_ecat/ecat_base.hpp"
#include "orthrus_ecat/ecat_typedef.hpp"
#include "orthrus_ecat/orthrus_leg.hpp"
#include "orthrus_ecat/orthrus_imu.hpp"

namespace orthrus_ecat
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

    orthrus_ecat::EcatBase Ethercat= orthrus_ecat::EcatBase(1); 

    // leg
    std::array<orthrus_ecat::Leg, 4> leg = {
        orthrus_ecat::Leg(CAN2,IMU1,USART1),
        orthrus_ecat::Leg(CAN2,IMU2,USART2),
        orthrus_ecat::Leg(CAN2,IMU3,USART3),
        orthrus_ecat::Leg(CAN2,IMU4,USART6)};

    // body imu
    orthrus_ecat::Imu body = orthrus_ecat::Imu(CAN2,IMU5);

  };
}