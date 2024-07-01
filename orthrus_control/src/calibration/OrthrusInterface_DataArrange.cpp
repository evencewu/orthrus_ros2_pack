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

        UnifiedSensorData();

        Eigen::Quaterniond q_relative = assembly_->body_imu.gyro_ * assembly_->body_imu.gyro_.inverse();

        Eigen::Matrix3d rotation_matrix = q_relative.toRotationMatrix();

        Eigen::Vector3d fix_angle_speed = rotation_matrix * assembly_->body_imu.angle_speed_;

        data[4] = fix_angle_speed[0];
        data[5] = fix_angle_speed[1];
        data[6] = fix_angle_speed[2];

        Eigen::Vector3d fix_acc = rotation_matrix * assembly_->body_imu.acc_;

        data[7] = fix_acc[0];
        data[8] = fix_acc[1];
        data[9] = fix_acc[2];

        data[3] = assembly_->body_imu.unified_gyro_.w();
        data[0] = assembly_->body_imu.unified_gyro_.x();
        data[1] = assembly_->body_imu.unified_gyro_.y();
        data[2] = assembly_->body_imu.unified_gyro_.z();

        //for (int i = 0; i < 4; i++)
        //{
        //    data[10 + i * 4] = assembly_->leg[i].imu.standard_gyro_.x();
        //    data[11 + i * 4] = assembly_->leg[i].imu.standard_gyro_.y();
        //    data[12 + i * 4] = assembly_->leg[i].imu.standard_gyro_.z();
        //    data[13 + i * 4] = assembly_->leg[i].imu.standard_gyro_.w();
        //}

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
}