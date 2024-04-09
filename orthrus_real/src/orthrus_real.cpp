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
    // ImuIfUseMag(FALSE);
  }

  void OrthrusInterfacesNode::MainLoop()
  {
    SetLED();
    SetLeg();

    Ethercat.EcatSyncMsg();
    AnalyzeAll();

    UnifiedSensorData();
    ImuTfPub();

    Log(LOG_FLAG);
  }

  void OrthrusInterfacesNode::Log(int flag)
  {
    if (flag == true)
    {
      RCLCPP_INFO(this->get_logger(), "%f", leg[0].motor[0].Pos_);
      RCLCPP_INFO(this->get_logger(), "%f", leg[0].motor[1].Pos_);
      RCLCPP_INFO(this->get_logger(), "=================");
      RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[0].imu.gyro_.w(), leg[0].imu.gyro_.x(), leg[0].imu.gyro_.y(), leg[0].imu.gyro_.z());
      RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[1].imu.gyro_.w(), leg[1].imu.gyro_.x(), leg[1].imu.gyro_.y(), leg[1].imu.gyro_.z());
      RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[2].imu.gyro_.w(), leg[2].imu.gyro_.x(), leg[2].imu.gyro_.y(), leg[3].imu.gyro_.z());
      RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[3].imu.gyro_.w(), leg[3].imu.gyro_.x(), leg[3].imu.gyro_.y(), leg[3].imu.gyro_.z());
      RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", body_imu.gyro_.w(), body_imu.gyro_.x(), body_imu.gyro_.y(), body_imu.gyro_.z());
      RCLCPP_INFO(this->get_logger(), "=================");
    }
  }

  void OrthrusInterfacesNode::SetLED()
  {
    if (led_flag_++ <= 10)
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
    if (motorcan_send_flag_ < 11)
    {
      leg[motorcan_send_flag_ / 3].motor[motorcan_send_flag_ % 3].SetOutput(&Ethercat.packet_tx[0], 0, 0, 0, 0, 0, 10);
      motorcan_send_flag_++;
    }
    else
    {
      leg[3].motor[2].SetOutput(&Ethercat.packet_tx[0], 0, 0, 0, 0, 0, 10);
      motorcan_send_flag_ = 0;
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

  /// @brief 统一imu姿态
  void OrthrusInterfacesNode::UnifiedSensorData()
  {
    body_imu.unified_gyro_ = calibrate::RotatingCoordinates(body_imu.gyro_, M_PI * 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI, Eigen::Vector3d(1.0, 0.0, 1.0));
    leg[0].imu.unified_gyro_ = calibrate::RotatingCoordinates(leg[0].imu.gyro_, -M_PI / 2, Eigen::Vector3d(1.0, 0.0, 0.0), M_PI * 2, Eigen::Vector3d(1.0, 1.0, 1.0));
    leg[1].imu.unified_gyro_ = calibrate::RotatingCoordinates(leg[1].imu.gyro_, -M_PI * 2, Eigen::Vector3d(0.0, 0.0, 1.0), M_PI * 2, Eigen::Vector3d(1.0, 1.0, 1.0));
    leg[2].imu.unified_gyro_ = calibrate::RotatingCoordinates(leg[2].imu.gyro_, -M_PI * 2, Eigen::Vector3d(0.0, 0.0, 1.0), M_PI * 2, Eigen::Vector3d(1.0, 1.0, 1.0));
    leg[3].imu.unified_gyro_ = calibrate::RotatingCoordinates(leg[3].imu.gyro_, -M_PI * 2, Eigen::Vector3d(0.0, 0.0, 1.0), M_PI * 2, Eigen::Vector3d(1.0, 1.0, 1.0));
  }

  /// @brief 用于程序退出前电机数据的处理
  void OrthrusInterfacesNode::SafeStop()
  {
    for (int i = 0; i <= 20; i++)
    {
      for (int j = 0; j < 12; j++)
      {
        leg[j / 3].motor[j % 3].SetOutput(&Ethercat.packet_tx[0], 0, 0, 0, 0, 0, 0);
        Ethercat.EcatSyncMsg();
      }
    }

    RCLCPP_INFO(this->get_logger(), "motor stop!");
  }

  /// @brief 发布imu的tf数据
  void OrthrusInterfacesNode::ImuTfPub()
  {
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
      tf_stamped.child_frame_id = "imu_1";

      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      tf_stamped.transform.rotation.w = leg[0].imu.unified_gyro_.w();
      tf_stamped.transform.rotation.x = leg[0].imu.unified_gyro_.x();
      tf_stamped.transform.rotation.y = leg[0].imu.unified_gyro_.y();
      tf_stamped.transform.rotation.z = leg[0].imu.unified_gyro_.z();

      if (leg[0].imu.unified_gyro_ != Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0))
      {
        orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);
      }

      tf_stamped.header.stamp = this->now();

      tf_stamped.header.frame_id = "horizontal";
      tf_stamped.child_frame_id = "imu_2";

      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      tf_stamped.transform.rotation.w = leg[1].imu.unified_gyro_.w();
      tf_stamped.transform.rotation.x = leg[1].imu.unified_gyro_.x();
      tf_stamped.transform.rotation.y = leg[1].imu.unified_gyro_.y();
      tf_stamped.transform.rotation.z = leg[1].imu.unified_gyro_.z();

      if (leg[1].imu.unified_gyro_ != Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0))
      {
        orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);
      }

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

      if (leg[2].imu.unified_gyro_ != Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0))
      {
        orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);
      }

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

      if (leg[3].imu.unified_gyro_ != Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0))
      {
        orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);
      }

      orthrus_calibrating_imu_pub_->publish(orthrus_calibrating_imu_msg_);

      imu_send_flag_ = 0;
    }
    else
    {
      imu_send_flag_++;
    }
  }

  /// @brief 是否启用磁力计
  /// @param flag 启用标志位
  void OrthrusInterfacesNode::ImuIfUseMag(bool flag)
  {
    if (flag)
    {
      Ethercat.packet_tx[0].can[CAN2].StdId = 0x11;
      Ethercat.packet_tx[0].can[CAN2].DLC = 0x04;
      Ethercat.packet_tx[0].can[CAN2].Data[0] = 0;
      Ethercat.packet_tx[0].can[CAN2].Data[1] = 0;
      Ethercat.packet_tx[0].can[CAN2].Data[2] = 0;
      Ethercat.packet_tx[0].can[CAN2].Data[3] = 1;
    }
    else
    {
      Ethercat.packet_tx[0].can[CAN2].StdId = 0x11;
      Ethercat.packet_tx[0].can[CAN2].DLC = 0x04;
      Ethercat.packet_tx[0].can[CAN2].Data[0] = 0;
      Ethercat.packet_tx[0].can[CAN2].Data[1] = 0;
      Ethercat.packet_tx[0].can[CAN2].Data[2] = 0;
      Ethercat.packet_tx[0].can[CAN2].Data[3] = 0;
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