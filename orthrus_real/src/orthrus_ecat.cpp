#include "orthrus_real/orthrus_ecat.hpp"

namespace orthrus_real
{
  OrthrusInterfacesNode::OrthrusInterfacesNode() : Node("orthrus_real")
  {
    init();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1), std::bind(&OrthrusInterfacesNode::main_loop, this));
  }

  OrthrusInterfacesNode::~OrthrusInterfacesNode()
  {
    safe_stop();
    Ethercat.EcatStop();
  }
  
  void OrthrusInterfacesNode::init()
  {
    char phy[] = "enp3s0";

    RCLCPP_INFO(this->get_logger(), "wl_driver启动,网口%s\n", phy);
    Ethercat.EcatStart(phy);
  }

  void OrthrusInterfacesNode::main_loop()
  {
    Ethercat.EcatSyncMsg();
    analyze_all();
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[0].imu.Gyro[0], leg[0].imu.Gyro[1], leg[3].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[1].imu.Gyro[0], leg[1].imu.Gyro[1], leg[3].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[2].imu.Gyro[0], leg[2].imu.Gyro[1], leg[3].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[3].imu.Gyro[0], leg[3].imu.Gyro[1], leg[3].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", body_imu.Gyro[0], body_imu.Gyro[1], body_imu.Gyro[2]);
  }

  void OrthrusInterfacesNode::safe_stop()
  {
    // TODO

    // safe stop code
  }

  void OrthrusInterfacesNode::analyze_all()
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