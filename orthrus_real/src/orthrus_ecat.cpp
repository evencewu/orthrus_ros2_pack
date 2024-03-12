#include "orthrus_real/orthrus_ecat.hpp"

namespace orthrus_real
{
  OrthrusInterfacesNode::OrthrusInterfacesNode() : Node("orthrus_real")
  {
    Init();

    timer_ = this->create_wall_timer(
        std::chrono::microseconds(100), std::bind(&OrthrusInterfacesNode::MainLoop, this));
  }

  OrthrusInterfacesNode::~OrthrusInterfacesNode()
  {
    SafeStop();
    Ethercat.EcatStop();
  }
  
  void OrthrusInterfacesNode::Init()
  {
    char phy[] = "enp5s0";

    RCLCPP_INFO(this->get_logger(), "wl_driver启动,网口%s\n", phy);
    Ethercat.EcatStart(phy);

    leg[0].init(CAN2,IMU1,USART1);
    leg[1].init(CAN2,IMU2,USART2);
    leg[2].init(CAN2,IMU3,USART3);
    leg[3].init(CAN2,IMU4,USART6);
    body_imu.init(CAN2,IMU5);
  }

  void OrthrusInterfacesNode::MainLoop()
  {
    Ethercat.EcatSyncMsg();
    AnalyzeAll();
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[0].imu.Gyro[0], leg[0].imu.Gyro[1], leg[0].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[1].imu.Gyro[0], leg[1].imu.Gyro[1], leg[1].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[2].imu.Gyro[0], leg[2].imu.Gyro[1], leg[2].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[3].imu.Gyro[0], leg[3].imu.Gyro[1], leg[3].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", body_imu.Gyro[0], body_imu.Gyro[1], body_imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "=================");
    RCLCPP_INFO(this->get_logger(), "motor %lf %lf %lf\n", Ethercat.packet_rx[0].Motor.motor_LW, Ethercat.packet_rx[0].Motor.motor_Pos, Ethercat.packet_rx[0].Motor.motor_W);
    RCLCPP_INFO(this->get_logger(), "=================");
    
  }

  void OrthrusInterfacesNode::SafeStop()
  {
    // TODO

    // safe stop code
  }

  void OrthrusInterfacesNode::AnalyzeAll()
  {
    leg[0].analyze(&Ethercat.packet_rx[0]);
    leg[1].analyze(&Ethercat.packet_rx[0]);
    leg[2].analyze(&Ethercat.packet_rx[0]);
    leg[3].analyze(&Ethercat.packet_rx[0]);
    body_imu.analyze(&Ethercat.packet_rx[0]);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orthrus_real::OrthrusInterfacesNode>());
  rclcpp::shutdown();
  return 0;
}