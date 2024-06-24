#pragma once

#include <inttypes.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "orthrus_ethercat/ethercat/TypeDef.hpp"

#define IMU1 0
#define IMU2 3
#define IMU3 6
#define IMU4 9
#define IMU5 12

#define YAW 0
#define PITCH 1
#define ROLL 2

namespace orthrus_ethercat
{
    class Imu
    {
    public:
        void Init(uint8_t device_id);
        void Analyze(Ecat_Inputs_Pack *pack);

        static void IfUseMag(bool flag, can_pack can);
        void RotatingCoordinates(double angle, Eigen::Vector3d axis, double install_angle, Eigen::Vector3d install_axis);

        void CorrectionMatrixSet(double angle1, Eigen::Vector3d axis1, double angle2, Eigen::Vector3d axis2);
        void GetEuler(Eigen::Quaterniond input_quaterniond);
        void Correction(double standard_yaw);

        Eigen::Quaterniond correction_matrix_;

        Eigen::Quaterniond gyro_;          // Raw gyro data
        Eigen::Quaterniond unified_gyro_;  // Correction direction
        Eigen::Quaterniond standard_gyro_; // Correction yaw

        Eigen::Vector3d euler_;

    private:
        union
        {
            float f_data;
            uint8_t uint_data[4];
        } data;

        uint8_t can_id_;
        uint8_t device_id_;
    };
}