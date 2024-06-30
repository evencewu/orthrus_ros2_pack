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
}