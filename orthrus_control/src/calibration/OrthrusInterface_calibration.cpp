#include "orthrus_control/OrthrusInterface.hpp"


namespace orthrus_control
{
    /// @brief 转换imu姿态
    void OrthrusSystemHardware::UnifiedSensorData()
    {
        body_imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
        leg[0].imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
        leg[1].imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
        leg[2].imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
        leg[3].imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));

        body_imu.Correction(body_imu.euler_(YAW));
        leg[0].imu.Correction(body_imu.euler_(YAW) - M_PI / 2);
        leg[1].imu.Correction(body_imu.euler_(YAW) + M_PI / 2);
        leg[2].imu.Correction(body_imu.euler_(YAW) - M_PI / 2);
        leg[3].imu.Correction(body_imu.euler_(YAW) + M_PI / 2);
    }

    /// @brief 通过发送Can消息开启编码器校准
    void OrthrusSystemHardware::StartCalibrationEncoderPosition()
    {
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "Start Calibration");
        Ethercat.packet_tx[0].can.StdId = 0x100;
        Ethercat.packet_tx[0].can.DLC = 0x04;
        Ethercat.packet_tx[0].can.Data[0] = 0; // 磁力计校准
        Ethercat.packet_tx[0].can.Data[1] = 1; // pos零点
        Ethercat.packet_tx[0].can.Data[2] = 0; // 静止校准
        Ethercat.packet_tx[0].can.Data[3] = 0; // enable磁力计

        Ethercat.packet_tx[1].can.StdId = 0x100;
        Ethercat.packet_tx[1].can.DLC = 0x04;
        Ethercat.packet_tx[1].can.Data[0] = 0; // 磁力计校准
        Ethercat.packet_tx[1].can.Data[1] = 1; // pos零点
        Ethercat.packet_tx[1].can.Data[2] = 0; // 静止校准
        Ethercat.packet_tx[1].can.Data[3] = 0; // enable磁力计
    }

    /// @brief 通过发送Can消息关闭编码器校准
    void OrthrusSystemHardware::StopCalibrationEncoderPosition()
    {
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "Stop Calibration");
        Ethercat.packet_tx[0].can.StdId = 0x100;
        Ethercat.packet_tx[0].can.DLC = 0x04;
        Ethercat.packet_tx[0].can.Data[0] = 0; // 磁力计校准
        Ethercat.packet_tx[0].can.Data[1] = 0; // pos零点
        Ethercat.packet_tx[0].can.Data[2] = 0; // 静止校准
        Ethercat.packet_tx[0].can.Data[3] = 0; // enable磁力计

        Ethercat.packet_tx[1].can.StdId = 0x100;
        Ethercat.packet_tx[1].can.DLC = 0x04;
        Ethercat.packet_tx[1].can.Data[0] = 0; // 磁力计校准
        Ethercat.packet_tx[1].can.Data[1] = 0; // pos零点
        Ethercat.packet_tx[1].can.Data[2] = 0; // 静止校准
        Ethercat.packet_tx[1].can.Data[3] = 0; // enable磁力计
    }
}