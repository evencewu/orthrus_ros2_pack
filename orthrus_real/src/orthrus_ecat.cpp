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

    leg[0].init(CAN2, IMU1, USART1);
    leg[1].init(CAN2, IMU2, USART2);
    leg[2].init(CAN2, IMU3, USART3);
    leg[3].init(CAN2, IMU4, USART6);
    body_imu.init(CAN2, IMU5);
  }

  void OrthrusInterfacesNode::MainLoop()
  {

    LedLoop();
    LegLoop();
    // Ethercat.EcatSyncMsg();
    // AnalyzeAll();

    // leg[0].motor[1].SetOutput(&Ethercat.packet_tx[0], 2, 0, 0, 0, 0, 0, 10);
    Ethercat.EcatSyncMsg();
    AnalyzeAll();

    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[0].imu.Gyro[0], leg[0].imu.Gyro[1], leg[0].imu.Gyro[2]);
    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[1].imu.Gyro[0], leg[1].imu.Gyro[1], leg[1].imu.Gyro[2]);
    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[2].imu.Gyro[0], leg[2].imu.Gyro[1], leg[2].imu.Gyro[2]);
    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[3].imu.Gyro[0], leg[3].imu.Gyro[1], leg[3].imu.Gyro[2]);
    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", body_imu.Gyro[0], body_imu.Gyro[1], body_imu.Gyro[2]);

    // RCLCPP_INFO(this->get_logger(), "=================");
    // RCLCPP_INFO(this->get_logger(), "motor %lf\n", (float)Ethercat.packet_rx[0].Motor.motor_id);
    // RCLCPP_INFO(this->get_logger(), "motor %lf\n", (float)Ethercat.packet_rx[0].Motor.motor_mode);
    // RCLCPP_INFO(this->get_logger(), "motor %lf\n", (float)Ethercat.packet_rx[0].Motor.motor_temp);
    // RCLCPP_INFO(this->get_logger(), "motor %lf\n", (float)Ethercat.packet_rx[0].Motor.motor_error);
    // RCLCPP_INFO(this->get_logger(), "motor %lf\n", (float)Ethercat.packet_rx[0].Motor.motor_T);
    // RCLCPP_INFO(this->get_logger(), "motor %lf\n", (float)Ethercat.packet_rx[0].Motor.motor_W);
    // RCLCPP_INFO(this->get_logger(), "=================");

    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].StdId);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].Data[0]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].Data[1]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].Data[2]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].Data[3]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].Data[4]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].Data[5]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].Data[6]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[1].Data[7]);
    //
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].StdId);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].Data[0]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].Data[1]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].Data[2]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].Data[3]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].Data[4]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].Data[5]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].Data[6]);
    // RCLCPP_INFO(this->get_logger(), "%d", Ethercat.packet_rx[0].can[0].Data[7]);

    RCLCPP_INFO(this->get_logger(), "=================");
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].StdId);
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].Data[0]);
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].Data[1]);
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].Data[2]);
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].Data[3]);
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].Data[4]);
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].Data[5]);
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].Data[6]);
    RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[1].Data[7]);
  }

  void OrthrusInterfacesNode::LedLoop()
  {
    if (led_flag_++ >= 10)
    {
      Ethercat.packet_tx[0].LED = 0x01;
      led_flag_ = 0;
    }
    else
    {
      Ethercat.packet_tx[0].LED = 0x00;
    }
  }

  void OrthrusInterfacesNode::LegLoop()
  {
    if (motorcan_send_flag_++ < 3)
    {
      leg[motorcan_send_flag_ / 9].motor[(motorcan_send_flag_ / 3) % 3].SetOutput(&Ethercat.packet_tx[0], motorcan_send_flag_ % 3, 0, 2, 0, 0, 0, 10);
    }
    else
    {
      motorcan_send_flag_ = 0;
    }
    motorcan_time_flag_ = 0;
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