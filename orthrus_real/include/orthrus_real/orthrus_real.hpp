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

#define LOG_FLAG 1

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

    void Log(int flag);

    /*Ecat数据写入*/
    void SetLED();
    void SetLeg();

    // Ecat code
    void SafeStop();
    void AnalyzeAll();
    
    // imu set
    void UnifiedSensorData();
    void ImuIfUseMag(bool flag);
    // Calibrating leg position
    void LegPositionCalibrate();

    /*实例化Ecat motor imu 结构体*/
    EcatBase Ethercat = EcatBase(1);
    Leg leg[4];
    Imu body_imu;

    /*ros2*/
    rclcpp::TimerBase::SharedPtr timer_;// ecat定时收发

    int led_flag_ = 0;
    int motor_send_flag_ = 0;
    int motorcan_send_flag_ = 0;


    

    // ros2
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr orthrus_imu_pub_;
    sensor_msgs::msg::Imu orthrus_imu_msg_;
    
    // Calibrating imu pub
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr orthrus_calibrating_imu_pub_;
    tf2_msgs::msg::TFMessage orthrus_calibrating_imu_msg_;
    int imu_send_flag_ = 0;
    void ImuTfPub();
  };
}