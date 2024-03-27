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
#include <cmath>

#include <Eigen/Dense>  
#include <Eigen/Geometry>  

#include "orthrus_real/ethercat/ecat_base.hpp"
#include "orthrus_real/ethercat/ecat_typedef.hpp"
#include "orthrus_real/calibrate/calibrate_imu.hpp"
#include "orthrus_real/calibrate/calibrate_leg.hpp"
#include "orthrus_real/orthrus_leg.hpp"
#include "orthrus_real/orthrus_imu.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

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
    void PubMsgLoop();
    
    void SetLED();
    void SetLeg();

    //Calibrating leg position 
    void LegPositionCalibrate();

    //Ecat code
    void SafeStop();
    void AnalyzeAll();

    EcatBase Ethercat = EcatBase(1);

    // leg
    Leg leg[4];
    // body imu
    Imu body_imu;
    // ros2 timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr msg_pub_timer_;

    int led_flag_ = 0;
    int motor_send_flag_ = 0;
    int motorcan_send_flag_ = 0;

    //ros2
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr orthrus_imu_pub_;
    sensor_msgs::msg::Imu orthrus_imu_msg_;
    int imu_send_flag_ = 0;

    //Calibrating imu pub
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr orthrus_calibrating_imu_pub_;
    tf2_msgs::msg::TFMessage orthrus_calibrating_imu_msg_;
  };
}