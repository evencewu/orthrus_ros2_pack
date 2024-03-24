#include "orthrus_real/orthrus_real.hpp"

namespace orthrus_real
{
  void get_angle(float q[4], float *yaw, float *pitch, float *roll)
  {
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
  }

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
  }

  void OrthrusInterfacesNode::MainLoop()
  {
    // LED
    SetLED();

    // Leg
    SetLeg();
    Ethercat.EcatSyncMsg();
    AnalyzeAll();

    // RCLCPP_INFO(this->get_logger(), "=================");
    // RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[0].Pos_);
    // RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[1].Pos_);
    // RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[2].Pos_);
    // RCLCPP_INFO(this->get_logger(), "=================");

    RCLCPP_INFO(this->get_logger(), "=================");
    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[0].imu.Gyro[0], leg[0].imu.Gyro[1], leg[0].imu.Gyro[2]);
    // RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[1].imu.Gyro[0], leg[1].imu.Gyro[1], leg[1].imu.Gyro[2]);
    //RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[2].imu.Gyro[0], leg[2].imu.Gyro[1], leg[2].imu.Gyro[2], leg[3].imu.Gyro[3]);
    //RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[3].imu.Gyro[0], leg[3].imu.Gyro[1], leg[3].imu.Gyro[2], leg[3].imu.Gyro[3]);
    //RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", body_imu.Gyro[0], body_imu.Gyro[1], body_imu.Gyro[2], body_imu.Gyro[3]);

    float yaw,pitch,roll;
    get_angle(body_imu.Gyro, &yaw,&pitch,&roll);

    RCLCPP_INFO(this->get_logger(), "imu %f %f %f\n", yaw,pitch,roll);

    if (imu_send_flag_ >= 10)
    {
      orthrus_imu_msg_.header.stamp = this->now();
      orthrus_imu_msg_.orientation.x = body_imu.Gyro[0];
      orthrus_imu_msg_.orientation.y = body_imu.Gyro[1];
      orthrus_imu_msg_.orientation.z = body_imu.Gyro[2];
      orthrus_imu_msg_.orientation.w = body_imu.Gyro[3];
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

      tf_stamped.transform.rotation.x = leg[2].imu.Gyro[0];
      tf_stamped.transform.rotation.y = leg[2].imu.Gyro[1];
      tf_stamped.transform.rotation.z = leg[2].imu.Gyro[2];
      tf_stamped.transform.rotation.w = leg[2].imu.Gyro[3];

      orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);

      tf_stamped.header.stamp = this->now();

      tf_stamped.header.frame_id = "horizontal";
      tf_stamped.child_frame_id = "imu_4";

      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      tf_stamped.transform.rotation.x = leg[3].imu.Gyro[0];
      tf_stamped.transform.rotation.y = leg[3].imu.Gyro[1];
      tf_stamped.transform.rotation.z = leg[3].imu.Gyro[2];
      tf_stamped.transform.rotation.w = leg[3].imu.Gyro[3];

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

  void OrthrusInterfacesNode::LegPositionCalibrating()
  {
    leg[0].motor[1].RealPosition_ = leg[0].motor[1].RealPosition_;
    leg[0].motor[2].RealPosition_ = leg[0].motor[2].RealPosition_;
    leg[0].motor[3].RealPosition_ = leg[0].motor[3].RealPosition_;

    leg[1].motor[1].RealPosition_ = leg[1].motor[1].RealPosition_;
    leg[1].motor[2].RealPosition_ = leg[1].motor[2].RealPosition_;
    leg[1].motor[3].RealPosition_ = leg[1].motor[3].RealPosition_;

    leg[2].motor[1].RealPosition_ = leg[2].motor[1].RealPosition_;
    leg[2].motor[2].RealPosition_ = leg[2].motor[2].RealPosition_;
    leg[2].motor[3].RealPosition_ = leg[2].motor[3].RealPosition_;

    leg[3].motor[1].RealPosition_ = leg[3].motor[1].RealPosition_;
    leg[3].motor[2].RealPosition_ = leg[3].motor[2].RealPosition_;
    leg[3].motor[3].RealPosition_ = leg[3].motor[3].RealPosition_;
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orthrus_real::OrthrusInterfacesNode>());
  rclcpp::shutdown();
  return 0;
}