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

    std::vector<double> OrthrusSystemHardware::PrepairSensorData()
    {
        std::vector<double> data(10,0.0);

        UnifiedSensorData();

        Eigen::Quaterniond q_relative = body_imu.gyro_ * body_imu.gyro_.inverse();

        Eigen::Matrix3d rotation_matrix = q_relative.toRotationMatrix();

        Eigen::Vector3d fix_angle_speed = rotation_matrix * body_imu.angle_speed_;

        data[4] = fix_angle_speed[0];
        data[5] = fix_angle_speed[1];
        data[6] = fix_angle_speed[2];

        Eigen::Vector3d fix_acc = rotation_matrix * body_imu.acc_;

        data[7] = fix_acc[0];
        data[8] = fix_acc[1];
        data[9] = fix_acc[2];

        data[3] = body_imu.unified_gyro_.w();
        data[0] = body_imu.unified_gyro_.x();
        data[1] = body_imu.unified_gyro_.y();
        data[2] = body_imu.unified_gyro_.z();

        return data;
    }

    std::vector<std::vector<double>> OrthrusSystemHardware::PrepairMotorData()
    {
        std::vector<double> position_data(12,0.0);
        std::vector<double> velocities_data(12,0.0);
        std::vector<double> acceleration_data(12,0.0);

        position_data[0] = -leg[0].motor[0].Pos_ / 9.1 - dealta_real_position_[0][0];
        position_data[1] = leg[0].motor[1].Pos_ / 9.1 - dealta_real_position_[0][1];
        position_data[2] = leg[0].motor[2].Pos_ / 9.1 - dealta_real_position_[0][2]; //

        position_data[3] = -leg[1].motor[0].Pos_ / 9.1 - dealta_real_position_[1][0];
        position_data[4] = leg[1].motor[1].Pos_ / 9.1 - dealta_real_position_[1][1];
        position_data[5] = leg[1].motor[2].Pos_ / 9.1 - dealta_real_position_[1][2];

        position_data[6] = -leg[2].motor[0].Pos_ / 9.1 - dealta_real_position_[2][0];
        position_data[7] = leg[2].motor[1].Pos_ / 9.1 - dealta_real_position_[2][1];
        position_data[8] = leg[2].motor[2].Pos_ / 9.1 - dealta_real_position_[2][2];

        position_data[9] = -leg[3].motor[0].Pos_ / 9.1 - dealta_real_position_[3][0];
        position_data[10] = leg[3].motor[1].Pos_ / 9.1 - dealta_real_position_[3][1];
        position_data[11] = leg[3].motor[2].Pos_ / 9.1 - dealta_real_position_[3][2];

        //velocities_data[0] = -leg[1].motor[0].W_;
        //acceleration_data[0] = leg[1].motor[0].Acc_;

        std::vector<std::vector<double>> data = {position_data, velocities_data, acceleration_data};
        return data;
    }
}