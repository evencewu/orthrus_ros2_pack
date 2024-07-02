#include "orthrus_control/OrthrusInterface.hpp"

namespace orthrus_control
{
    /// @brief 转换imu姿态
    void OrthrusSystemHardware::UnifiedSensorData()
    {
        assembly_->body_imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
        assembly_->leg[0].imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
        assembly_->leg[1].imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
        assembly_->leg[2].imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));
        assembly_->leg[3].imu.CorrectionMatrixSet(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0), -M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0));

        assembly_->body_imu.Correction(assembly_->body_imu.euler_(YAW));
        assembly_->leg[0].imu.Correction(assembly_->body_imu.euler_(YAW) - M_PI / 2);
        assembly_->leg[1].imu.Correction(assembly_->body_imu.euler_(YAW) + M_PI / 2);
        assembly_->leg[2].imu.Correction(assembly_->body_imu.euler_(YAW) - M_PI / 2);
        assembly_->leg[3].imu.Correction(assembly_->body_imu.euler_(YAW) + M_PI / 2);
    }

    std::vector<double> OrthrusSystemHardware::PrepairSensorData()
    {
        std::vector<double> data(10, 0.0);

        // BodyImufliter();

        UnifiedSensorData();

        for (int i = 0; i < 10; i++)
        {
            data[i] = 1;
        }

        /*
        data[20] = assembly_->body_imu.standard_angle_speed_[0];
        data[21] = assembly_->body_imu.standard_angle_speed_[1];
        data[22] = assembly_->body_imu.standard_angle_speed_[2];

        data[23] = assembly_->body_imu.standard_acc_[0];
        data[24] = assembly_->body_imu.standard_acc_[1];
        data[25] = assembly_->body_imu.standard_acc_[2];

        data[19] = assembly_->body_imu.unified_gyro_.w();
        data[16] = assembly_->body_imu.unified_gyro_.x();
        data[17] = assembly_->body_imu.unified_gyro_.y();
        data[18] = assembly_->body_imu.unified_gyro_.z();

        for (int i = 0; i < 4; i++)
        {
            data[0 + i * 4] = assembly_->leg[i].imu.standard_gyro_.x();
            data[1 + i * 4] = assembly_->leg[i].imu.standard_gyro_.y();
            data[2 + i * 4] = assembly_->leg[i].imu.standard_gyro_.z();
            data[3 + i * 4] = assembly_->leg[i].imu.standard_gyro_.w();
        }
        */

        return data;
    }

    std::vector<std::vector<double>> OrthrusSystemHardware::PrepairMotorData()
    {
        std::vector<double> position_data(12, 0.0);
        std::vector<double> velocities_data(12, 0.0);
        std::vector<double> acceleration_data(12, 0.0);

        for (int i = 0; i < 4; i++)
        {

            position_data[i * 3 + 0] = -assembly_->leg[i].motor[0].Pos_ / 9.1 - dealta_real_position_[i][0];
            position_data[i * 3 + 1] = assembly_->leg[i].motor[1].Pos_ / 9.1 - dealta_real_position_[i][1];
            position_data[i * 3 + 2] = assembly_->leg[i].motor[2].Pos_ / 9.1 - dealta_real_position_[i][2];
        }

        // velocities_data[0] = -leg[1].motor[0].W_;
        // acceleration_data[0] = leg[1].motor[0].Acc_;

        std::vector<std::vector<double>> data = {position_data, velocities_data, acceleration_data};
        return data;
    }

    void OrthrusSystemHardware::BodyImufliter()
    {
        double list[10];

        imufliter_list_[9] = imufliter_list_[8];
        imufliter_list_[8] = imufliter_list_[7];
        imufliter_list_[7] = imufliter_list_[6];
        imufliter_list_[6] = imufliter_list_[5];
        imufliter_list_[5] = imufliter_list_[4];
        imufliter_list_[4] = imufliter_list_[3];
        imufliter_list_[3] = imufliter_list_[2];
        imufliter_list_[2] = imufliter_list_[1];
        imufliter_list_[1] = imufliter_list_[0];
        imufliter_list_[0] = assembly_->body_imu.unified_gyro_.y();

        for (int i = 0; i < 5; i++)
        {
            list[i] = imufliter_list_[i];
        }

        std::sort(std::begin(list), std::end(list));

        assembly_->body_imu.unified_gyro_.y() = list[5];
    }
}