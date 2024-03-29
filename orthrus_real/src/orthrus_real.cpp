#include "orthrus_real/orthrus_real.hpp"

namespace orthrus_real
{
  OrthrusInterfacesNode::OrthrusInterfacesNode() : Node("orthrus_real")
  {
    Init();

    orthrus_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/orthrus_interface/imu", 10);

    orthrus_calibrating_imu_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

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

    ImuIfUseMag(TRUE);
    //ImuIfUseMag(FALSE);
  }

  void OrthrusInterfacesNode::MainLoop()
  {
    // LED
    SetLED();

    // Leg
    // SetLeg();
    Ethercat.EcatSyncMsg();
    AnalyzeAll();
    UnifiedSensorData();

    // RCLCPP_INFO(this->get_logger(), "=================");
    // RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[0].Pos_);
    // RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[1].Pos_);
    // RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[2].Pos_);
    // RCLCPP_INFO(this->get_logger(), "=================");

    RCLCPP_INFO(this->get_logger(), "=================");
    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[0].imu.Gyro[0], leg[0].imu.Gyro[1], leg[0].imu.Gyro[2]);
    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[1].imu.Gyro[0], leg[1].imu.Gyro[1], leg[1].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[2].imu.gyro_.w(), leg[2].imu.gyro_.x(), leg[2].imu.gyro_.y(), leg[3].imu.gyro_.z());
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[3].imu.gyro_.w(), leg[3].imu.gyro_.x(), leg[3].imu.gyro_.y(), leg[3].imu.gyro_.z());
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", body_imu.gyro_.w(), body_imu.gyro_.x(), body_imu.gyro_.y(), body_imu.gyro_.z());

    RCLCPP_INFO(this->get_logger(), "=================");

    if (imu_send_flag_ >= 10)
    {
      orthrus_imu_msg_.header.stamp = this->now();
      orthrus_imu_msg_.orientation.w = body_imu.unified_gyro_.w();
      orthrus_imu_msg_.orientation.x = body_imu.unified_gyro_.x();
      orthrus_imu_msg_.orientation.y = body_imu.unified_gyro_.y();
      orthrus_imu_msg_.orientation.z = body_imu.unified_gyro_.z();
      orthrus_imu_pub_->publish(orthrus_imu_msg_);

      geometry_msgs::msg::TransformStamped tf_stamped;
      // orthrus_viewer_horizontal_pub_

      orthrus_calibrating_imu_msg_.transforms.clear();

      tf_stamped.header.stamp = this->now();

      tf_stamped.header.frame_id = "horizontal";
      tf_stamped.child_frame_id = "imu_3";

      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      tf_stamped.transform.rotation.w = leg[2].imu.unified_gyro_.w();
      tf_stamped.transform.rotation.x = leg[2].imu.unified_gyro_.x();
      tf_stamped.transform.rotation.y = leg[2].imu.unified_gyro_.y();
      tf_stamped.transform.rotation.z = leg[2].imu.unified_gyro_.z();

      orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);

      tf_stamped.header.stamp = this->now();

      tf_stamped.header.frame_id = "horizontal";
      tf_stamped.child_frame_id = "imu_4";

      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      tf_stamped.transform.rotation.w = leg[3].imu.unified_gyro_.w();
      tf_stamped.transform.rotation.x = leg[3].imu.unified_gyro_.x();
      tf_stamped.transform.rotation.y = leg[3].imu.unified_gyro_.y();
      tf_stamped.transform.rotation.z = leg[3].imu.unified_gyro_.z();

      orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);

      orthrus_calibrating_imu_pub_->publish(orthrus_calibrating_imu_msg_);

      imu_send_flag_ = 0;
    }
    else
    {
      imu_send_flag_++;
    }

    // RCLCPP_INFO(this->get_logger(), "=================");

    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].StdId);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].Data[0]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].Data[1]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].Data[2]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].Data[3]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].Data[4]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].Data[5]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].Data[6]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_tx[0].can[0].Data[7]);

    // RCLCPP_INFO(this->get_logger(), "=================");
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].StdId);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].DLC);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].Data[0]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].Data[1]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].Data[2]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].Data[3]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].Data[4]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].Data[5]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].Data[6]);
    // RCLCPP_INFO(this->get_logger(), "0x%x", Ethercat.packet_rx[0].can[1].Data[7]);
    // RCLCPP_INFO(this->get_logger(), "=================");
  }

  void OrthrusInterfacesNode::SetLED()
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

  void OrthrusInterfacesNode::SetLeg()
  {
    if (motorcan_send_flag_++ < 9)
    {
      leg[motorcan_send_flag_ / 9].motor[(motorcan_send_flag_ / 3) % 3].SetOutput(&Ethercat.packet_tx[0], motorcan_send_flag_ % 3, 0, 0, 0, 0, 0, 10);
    }
    else
    {
      motorcan_send_flag_ = -1;
    }
  }

  void OrthrusInterfacesNode::AnalyzeAll()
  {
    leg[0].analyze(&Ethercat.packet_rx[0]);
    leg[1].analyze(&Ethercat.packet_rx[0]);
    leg[2].analyze(&Ethercat.packet_rx[0]);
    leg[3].analyze(&Ethercat.packet_rx[0]);
    body_imu.analyze(&Ethercat.packet_rx[0]);
  }

  void OrthrusInterfacesNode::UnifiedSensorData()
  {
    body_imu.unified_gyro_ = calibrate::RotatingCoordinates(body_imu.gyro_, M_PI * 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0));
    leg[2].imu.unified_gyro_ = calibrate::RotatingCoordinates(leg[2].imu.gyro_, -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0), M_PI * 2 / 3, Eigen::Vector3d(1.0, 1.0, 1.0));
    leg[3].imu.unified_gyro_ = calibrate::RotatingCoordinates(leg[3].imu.gyro_, -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0), M_PI * 2 / 3, Eigen::Vector3d(1.0, 1.0, 1.0));
  }

  void OrthrusInterfacesNode::SafeStop()
  {
    // TODO

    // safe stop code
  }

  void OrthrusInterfacesNode::ImuIfUseMag(bool flag)
  {
    if (flag)
    {
      Ethercat.packet_tx[0].can[0].StdId = 0x11;
      Ethercat.packet_tx[0].can[0].DLC = 0x04;
      Ethercat.packet_tx[0].can[0].Data[0] = 0;
      Ethercat.packet_tx[0].can[0].Data[1] = 0;
      Ethercat.packet_tx[0].can[0].Data[2] = 0;
      Ethercat.packet_tx[0].can[0].Data[3] = 1;

      Ethercat.packet_tx[0].can[1].StdId = 0x11;
      Ethercat.packet_tx[0].can[1].DLC = 0x04;
      Ethercat.packet_tx[0].can[1].Data[0] = 0;
      Ethercat.packet_tx[0].can[1].Data[1] = 0;
      Ethercat.packet_tx[0].can[1].Data[2] = 0;
      Ethercat.packet_tx[0].can[1].Data[3] = 1;
    }
    else
    {
      Ethercat.packet_tx[0].can[0].StdId = 0x11;
      Ethercat.packet_tx[0].can[0].DLC = 0x04;
      Ethercat.packet_tx[0].can[0].Data[0] = 0;
      Ethercat.packet_tx[0].can[0].Data[1] = 0;
      Ethercat.packet_tx[0].can[0].Data[2] = 0;
      Ethercat.packet_tx[0].can[0].Data[3] = 0;

      Ethercat.packet_tx[0].can[1].StdId = 0x11;
      Ethercat.packet_tx[0].can[1].DLC = 0x04;
      Ethercat.packet_tx[0].can[1].Data[0] = 0;
      Ethercat.packet_tx[0].can[1].Data[1] = 0;
      Ethercat.packet_tx[0].can[1].Data[2] = 0;
      Ethercat.packet_tx[0].can[1].Data[3] = 0;
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orthrus_real::OrthrusInterfacesNode>());
  rclcpp::shutdown();
  return 0;
}