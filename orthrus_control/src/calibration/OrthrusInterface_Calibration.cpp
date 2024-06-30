#include "orthrus_control/OrthrusInterface.hpp"

namespace orthrus_control
{
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

    void OrthrusSystemHardware::CalibrationPosition()
    {
        double imu_pos_0;
        double imu_pos_1;

        imu_pos_0 = -(assembly_->body_imu.euler_(ROLL) - assembly_->leg[0].imu.euler_(PITCH));
        imu_pos_1 = (assembly_->body_imu.euler_(PITCH) - assembly_->leg[0].imu.euler_(ROLL)) + theta2 * M_PI / 180;

        dealta_real_position_[0][0] = -assembly_->leg[0].motor[0].Pos_ / 9.1 + imu_pos_0;
        dealta_real_position_[0][1] = assembly_->leg[0].motor[1].Pos_ / 9.1 - imu_pos_1;
        dealta_real_position_[0][2] = assembly_->leg[0].motor[2].Pos_ / 9.1 - assembly_->leg[0].angle.Pos_ - (30 * M_PI / 180 - theta1 * M_PI / 180);

        imu_pos_0 = (assembly_->body_imu.euler_(ROLL) - assembly_->leg[1].imu.euler_(PITCH));
        imu_pos_1 = (assembly_->body_imu.euler_(PITCH) - assembly_->leg[1].imu.euler_(ROLL)) - theta2 * M_PI / 180;

        dealta_real_position_[1][0] = -assembly_->leg[1].motor[0].Pos_ / 9.1 + imu_pos_0;
        dealta_real_position_[1][1] = assembly_->leg[1].motor[1].Pos_ / 9.1 - imu_pos_1;
        dealta_real_position_[1][2] = assembly_->leg[1].motor[2].Pos_ / 9.1 - assembly_->leg[1].angle.Pos_ + (30 * M_PI / 180 - theta1 * M_PI / 180);

        imu_pos_0 = assembly_->body_imu.euler_(ROLL) - assembly_->leg[2].imu.euler_(PITCH);
        imu_pos_1 = assembly_->body_imu.euler_(PITCH) - assembly_->leg[2].imu.euler_(ROLL) + theta2 * M_PI / 180;

        dealta_real_position_[2][0] = -assembly_->leg[2].motor[0].Pos_ / 9.1 + imu_pos_0;
        dealta_real_position_[2][1] = assembly_->leg[2].motor[1].Pos_ / 9.1 - imu_pos_1;
        dealta_real_position_[2][2] = assembly_->leg[2].motor[2].Pos_ / 9.1 - assembly_->leg[2].angle.Pos_ - (30 * M_PI / 180 - theta1 * M_PI / 180); //

        imu_pos_0 = -(assembly_->body_imu.euler_(ROLL) - assembly_->leg[3].imu.euler_(PITCH));
        imu_pos_1 = (assembly_->body_imu.euler_(PITCH) - assembly_->leg[3].imu.euler_(ROLL)) - theta2 * M_PI / 180;

        dealta_real_position_[3][0] = -assembly_->leg[3].motor[0].Pos_ / 9.1 + imu_pos_0;
        dealta_real_position_[3][1] = assembly_->leg[3].motor[1].Pos_ / 9.1 - imu_pos_1;
        dealta_real_position_[3][2] = assembly_->leg[3].motor[2].Pos_ / 9.1 - assembly_->leg[3].angle.Pos_ + (30 * M_PI / 180 - theta1 * M_PI / 180);
    }

    std::vector<double> OrthrusSystemHardware::CompensationAngleError()
    {
        std::vector<double> positions(12, 0.0);

        return positions;
    }
}