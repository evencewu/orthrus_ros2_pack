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

        data[4] = assembly_->body_imu.standard_angle_speed_[0];
        data[5] = assembly_->body_imu.standard_angle_speed_[1];
        data[6] = assembly_->body_imu.standard_angle_speed_[2];

        data[7] = assembly_->body_imu.standard_acc_[0];
        data[8] = assembly_->body_imu.standard_acc_[1];
        data[9] = assembly_->body_imu.standard_acc_[2];

        data[3] = assembly_->body_imu.unified_gyro_.w();
        data[0] = assembly_->body_imu.unified_gyro_.x();
        data[1] = assembly_->body_imu.unified_gyro_.y();
        data[2] = assembly_->body_imu.unified_gyro_.z();

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