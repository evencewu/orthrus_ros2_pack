#include "orthrus_real/orthrus_real.hpp"

namespace orthrus_real
{
  void get_angle(Eigen::Quaterniond q, double *yaw, double *pitch, double *roll)
  {
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

    // 从旋转矩阵中提取欧拉角（这里使用Z-Y-X顺序）

    // 如果sin(pitch)不为零，则可以避免除以零的情况
    if (std::abs(rotationMatrix(2, 0)) != 1)
    {
      *roll = std::atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
      *pitch = std::asin(-rotationMatrix(2, 0));
      *yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    }
    else
    {
      // 当sin(pitch)接近零时，情况变得不确定，但我们仍然可以通过其他方式获取合理的解
      // 在这种情况下，我们可以选择roll和yaw的任意值，因为pitch已经是90度或-90度
      // 这里我们简单地将roll和yaw设为0
      *roll  = 0;
      *pitch = std::copysign(M_PI / 2, -rotationMatrix(2, 0));
      *yaw = 0;
    }
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
    //SetLeg();
    Ethercat.EcatSyncMsg();
    AnalyzeAll();

    RCLCPP_INFO(this->get_logger(), "=================");
    RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[0].Pos_);
    RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[1].Pos_);
    RCLCPP_INFO(this->get_logger(), "0x%f", leg[0].motor[2].Pos_);
    RCLCPP_INFO(this->get_logger(), "=================");

    RCLCPP_INFO(this->get_logger(), "=================");
    //RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[0].imu.Gyro[0], leg[0].imu.Gyro[1], leg[0].imu.Gyro[2]);
    //RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf\n", leg[1].imu.Gyro[0], leg[1].imu.Gyro[1], leg[1].imu.Gyro[2]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[2].imu.Gyro[0], leg[2].imu.Gyro[1], leg[2].imu.Gyro[2], leg[3].imu.Gyro[3]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", leg[3].imu.Gyro[0], leg[3].imu.Gyro[1], leg[3].imu.Gyro[2], leg[3].imu.Gyro[3]);
    RCLCPP_INFO(this->get_logger(), "imu %lf %lf %lf %lf\n", body_imu.Gyro[0], body_imu.Gyro[1], body_imu.Gyro[2], body_imu.Gyro[3]);

    Eigen::AngleAxisd rotation_y(M_PI / 2, Eigen::Vector3d(0, 1, 0)); // 绕y轴旋转90度
    Eigen::Quaterniond q_y(rotation_y);

    Eigen::AngleAxisd rotation_x(M_PI / 2, Eigen::Vector3d(1, 0, 0)); // 绕x轴旋转90度
    Eigen::Quaterniond q_x(rotation_x);

    Eigen::AngleAxisd rotation_z(M_PI / 2, Eigen::Vector3d(0, 0, 1)); // 绕z轴旋转90度
    Eigen::Quaterniond q_z(rotation_z);
    // 将AngleAxisd转换为四元数
    
    Eigen::Quaterniond rotation_eigen{body_imu.Gyro[0], body_imu.Gyro[1], body_imu.Gyro[2], body_imu.Gyro[3]};
    Eigen::Quaterniond body = rotation_eigen * q_y;

    Eigen::Quaterniond rotation_eigen_1{leg[2].imu.Gyro[0], leg[2].imu.Gyro[1], leg[2].imu.Gyro[2], leg[2].imu.Gyro[3]};
    Eigen::Quaterniond leg_3 = rotation_eigen_1;

    Eigen::Quaterniond rotation_eigen_2{leg[3].imu.Gyro[0], leg[3].imu.Gyro[1], leg[3].imu.Gyro[2], leg[3].imu.Gyro[3]};
    Eigen::Quaterniond leg_4 = rotation_eigen_2;

    double yaw, pitch, roll;
    get_angle(body, &yaw, &pitch, &roll);
    RCLCPP_INFO(this->get_logger(), "imu %f %f %f\n", yaw, pitch, roll);
    RCLCPP_INFO(this->get_logger(), "=================");

    if (imu_send_flag_ >= 10)
    {
      orthrus_imu_msg_.header.stamp = this->now();
      orthrus_imu_msg_.orientation.w = body.w();
      orthrus_imu_msg_.orientation.x = body.x();
      orthrus_imu_msg_.orientation.y = body.y();
      orthrus_imu_msg_.orientation.z = body.z();
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

      tf_stamped.transform.rotation.w = leg_3.w();
      tf_stamped.transform.rotation.x = leg_3.x();
      tf_stamped.transform.rotation.y = leg_3.y();
      tf_stamped.transform.rotation.z = leg_3.z();

      orthrus_calibrating_imu_msg_.transforms.push_back(tf_stamped);

      tf_stamped.header.stamp = this->now();

      tf_stamped.header.frame_id = "horizontal";
      tf_stamped.child_frame_id = "imu_4";

      tf_stamped.transform.translation.x = 0.0;
      tf_stamped.transform.translation.y = 0.0;
      tf_stamped.transform.translation.z = 0.0;

      tf_stamped.transform.rotation.w = leg_4.w();
      tf_stamped.transform.rotation.x = leg_4.x();
      tf_stamped.transform.rotation.y = leg_4.y();
      tf_stamped.transform.rotation.z = leg_4.z();

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

  void OrthrusInterfacesNode::LegPositionCalibrate()
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

  void OrthrusInterfacesNode::ImuCalibrate(Ecat_Outputs_Pack *pack,int can_port)
  {
    pack->can[can_port].StdId = 0x11;
    pack->can[can_port].DLC = 0x08;
    pack->can[can_port].Data[0] = 1;
    pack->can[can_port].Data[1] = 1;
    pack->can[can_port].Data[2] = 1;
    pack->can[can_port].Data[3] = 1;
    pack->can[can_port].Data[4] = 1;
    pack->can[can_port].Data[5] = 1;
    pack->can[can_port].Data[6] = 1;
    pack->can[can_port].Data[7] = 1;
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orthrus_real::OrthrusInterfacesNode>());
  rclcpp::shutdown();
  return 0;
}