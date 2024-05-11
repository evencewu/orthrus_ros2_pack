#include "orthrus_real/orthrus_real.hpp"

namespace orthrus_real
{
  OrthrusInterfacesNode::OrthrusInterfacesNode() : Node("orthrus_real")
  {
    Init();

    orthrus_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/orthrus_interface/imu", 10);

    orthrus_calibrating_imu_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

    timer_ = this->create_wall_timer(std::chrono::microseconds(100), std::bind(&OrthrusInterfacesNode::MainLoop, this));
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

    leg[0].Init(IMU1, USART1);
    leg[1].Init(IMU2, USART2);
    leg[2].Init(IMU4, USART3);
    leg[3].Init(IMU3, USART6);
    body_imu.Init(IMU5);

    body_imu.CorrectionMatrixSet(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
    leg[0].imu.CorrectionMatrixSet(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
    leg[1].imu.CorrectionMatrixSet(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
    leg[2].imu.CorrectionMatrixSet(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
    leg[3].imu.CorrectionMatrixSet(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));

    Imu::IfUseMag(FALSE, Ethercat.packet_rx[0].can);
    Imu::IfUseMag(FALSE, Ethercat.packet_rx[1].can);
    Ethercat.EcatSyncMsg();

    // ImuIfUseMag(FALSE);
  }

  void OrthrusInterfacesNode::MainLoop()
  {
    SetLED(1000);
    SetLeg();

    Ethercat.EcatSyncMsg();
    AnalyzeAll();

    LegPositionCalibrate();

    ImuTfPub();

    Log(LOG_FLAG);
  }

  void OrthrusInterfacesNode::Log(int flag)
  {
    if (flag == true)
    {
      // RCLCPP_INFO(this->get_logger(), "=================");
      // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf", leg[0].imu.standard_gyro_.w(), leg[0].imu.standard_gyro_.x(), leg[0].imu.standard_gyro_.y(), leg[0].imu.standard_gyro_.z());
      // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf", leg[1].imu.standard_gyro_.w(), leg[1].imu.standard_gyro_.x(), leg[1].imu.standard_gyro_.y(), leg[1].imu.standard_gyro_.z());
      // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf", leg[2].imu.standard_gyro_.w(), leg[2].imu.standard_gyro_.x(), leg[2].imu.standard_gyro_.y(), leg[3].imu.standard_gyro_.z());
      // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf", leg[3].imu.standard_gyro_.w(), leg[3].imu.standard_gyro_.x(), leg[3].imu.standard_gyro_.y(), leg[3].imu.standard_gyro_.z());
      // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf", body_imu.standard_gyro_.w(), body_imu.standard_gyro_.x(), body_imu.standard_gyro_.y(), body_imu.standard_gyro_.z());
      // RCLCPP_INFO(this->get_logger(), "=================");

      RCLCPP_INFO(this->get_logger(), "=================");
      RCLCPP_INFO(this->get_logger(), "\n motor1 %lf %lf %lf %lf \n motor2 %lf %lf %lf %lf ",
                  body_imu.euler_(PITCH) - leg[0].imu.euler_(ROLL),
                  body_imu.euler_(PITCH) - leg[1].imu.euler_(ROLL),
                  body_imu.euler_(PITCH) - leg[2].imu.euler_(ROLL),
                  body_imu.euler_(PITCH) - leg[3].imu.euler_(ROLL),
                  body_imu.euler_(ROLL) - leg[0].imu.euler_(PITCH),
                  body_imu.euler_(ROLL) - leg[1].imu.euler_(PITCH),
                  body_imu.euler_(ROLL) - leg[2].imu.euler_(PITCH),
                  body_imu.euler_(ROLL) - leg[3].imu.euler_(PITCH));
      RCLCPP_INFO(this->get_logger(), "\n motor0 %lf %lf %lf %lf", leg[0].motor[0].Pos_, leg[1].motor[0].Pos_, leg[2].motor[0].Pos_, leg[3].motor[0].Pos_);
      RCLCPP_INFO(this->get_logger(), "=================");

      // RCLCPP_INFO(this->get_logger(), "=================");
      // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n %lf %lf %lf\n %lf %lf %lf\n %lf %lf %lf\n",
      //             leg[0].imu.euler_(YAW), leg[0].imu.euler_(PITCH), leg[0].imu.euler_(ROLL),
      //             leg[1].imu.euler_(YAW), leg[1].imu.euler_(PITCH), leg[1].imu.euler_(ROLL),
      //             leg[2].imu.euler_(YAW), leg[2].imu.euler_(PITCH), leg[2].imu.euler_(ROLL),
      //             leg[3].imu.euler_(YAW), leg[3].imu.euler_(PITCH), leg[3].imu.euler_(ROLL));
      // RCLCPP_INFO(this->get_logger(), "=================");
    }
  }

  void OrthrusInterfacesNode::SetLED(int frequency_division)
  {
    if (led_flag_ < frequency_division / 2)
    {
      Ethercat.packet_tx[0].LED = 0x02;
      Ethercat.packet_tx[1].LED = 0x02;
      led_flag_++;
    }
    else if (led_flag_ < frequency_division)
    {
      Ethercat.packet_tx[0].LED = 0x05;
      Ethercat.packet_tx[1].LED = 0x05;
      led_flag_++;
    }
    else
    {
      led_flag_ = 0;
    }
  }

  void OrthrusInterfacesNode::SetLeg()
  {
    if (motorcan_send_flag_ < 6)
    {
      leg[motorcan_send_flag_ / 3].motor[motorcan_send_flag_ % 3].SetOutput(&Ethercat.packet_tx[0], 0, 0, 0, 0, 0, 10);
      motorcan_send_flag_++;
    }
    else if (motorcan_send_flag_ < 11 && motorcan_send_flag_ >= 6)
    {
      leg[motorcan_send_flag_ / 3].motor[motorcan_send_flag_ % 3].SetOutput(&Ethercat.packet_tx[1], 0, 0, 0, 0, 0, 10);
      motorcan_send_flag_++;
    }
    else
    {
      leg[3].motor[2].SetOutput(&Ethercat.packet_tx[1], 0, 0, 0, 0, 0, 10);
      motorcan_send_flag_ = 0;
    }
  }

  void OrthrusInterfacesNode::AnalyzeAll()
  {
    leg[0].Analyze(&Ethercat.packet_rx[0]);
    leg[1].Analyze(&Ethercat.packet_rx[0]);
    leg[2].Analyze(&Ethercat.packet_rx[1]);
    leg[3].Analyze(&Ethercat.packet_rx[1]);
    body_imu.Analyze(&Ethercat.packet_rx[1]);
  }

  void OrthrusInterfacesNode::LegPositionCalibrate()
  {
    UnifiedSensorData();
  }

  /// @brief 统一imu姿态
  void OrthrusInterfacesNode::UnifiedSensorData()
  {
    body_imu.RotatingCoordinates(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
    leg[0].imu.RotatingCoordinates(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
    leg[1].imu.RotatingCoordinates(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
    leg[2].imu.RotatingCoordinates(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
    leg[3].imu.RotatingCoordinates(-M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));

    body_imu.Correction(body_imu.euler_(YAW));
    leg[0].imu.Correction(body_imu.euler_(YAW) - M_PI / 2);
    leg[1].imu.Correction(body_imu.euler_(YAW) + M_PI / 2);
    leg[2].imu.Correction(body_imu.euler_(YAW) - M_PI / 2);
    leg[3].imu.Correction(body_imu.euler_(YAW) + M_PI / 2);
  }

  /// @brief 用于程序退出前电机数据的处理
  void OrthrusInterfacesNode::SafeStop()
  {
    for (int i = 0; i <= 20; i++)
    {
      for (int j = 0; j < 12; j++)
      {
        if (j > 5)
        {
          leg[j / 3].motor[j % 3].SetOutput(&Ethercat.packet_tx[1], 0, 0, 0, 0, 0, 0);
          Ethercat.EcatSyncMsg();
        }
        else
        {
          leg[j / 3].motor[j % 3].SetOutput(&Ethercat.packet_tx[0], 0, 0, 0, 0, 0, 0);
          Ethercat.EcatSyncMsg();
        }
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

      if (PUB_LEGIMU_FLAG == 1)
      {
        geometry_msgs::msg::TransformStamped tf_stamped;

        orthrus_calibrating_imu_msg_.transforms.clear();

        tf_stamped.header.stamp = this->now();

        tf_stamped.header.frame_id = "horizontal";
        tf_stamped.child_frame_id = "imu_1";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;

        tf_stamped.transform.rotation.w = leg[0].imu.standard_gyro_.w();
        tf_stamped.transform.rotation.x = leg[0].imu.standard_gyro_.x();
        tf_stamped.transform.rotation.y = leg[0].imu.standard_gyro_.y();
        tf_stamped.transform.rotation.z = leg[0].imu.standard_gyro_.z();

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

        tf_stamped.transform.rotation.w = leg[1].imu.standard_gyro_.w();
        tf_stamped.transform.rotation.x = leg[1].imu.standard_gyro_.x();
        tf_stamped.transform.rotation.y = leg[1].imu.standard_gyro_.y();
        tf_stamped.transform.rotation.z = leg[1].imu.standard_gyro_.z();

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

        tf_stamped.transform.rotation.w = leg[2].imu.standard_gyro_.w();
        tf_stamped.transform.rotation.x = leg[2].imu.standard_gyro_.x();
        tf_stamped.transform.rotation.y = leg[2].imu.standard_gyro_.y();
        tf_stamped.transform.rotation.z = leg[2].imu.standard_gyro_.z();

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

        tf_stamped.transform.rotation.w = leg[3].imu.standard_gyro_.w();
        tf_stamped.transform.rotation.x = leg[3].imu.standard_gyro_.x();
        tf_stamped.transform.rotation.y = leg[3].imu.standard_gyro_.y();
        tf_stamped.transform.rotation.z = leg[3].imu.standard_gyro_.z();

        if (leg[3].imu.unified_gyro_ != Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0))
        {
          orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);
        }

        orthrus_calibrating_imu_pub_->publish(orthrus_calibrating_imu_msg_);
      }

      imu_send_flag_ = 0;
    }
    else
    {
      imu_send_flag_++;
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