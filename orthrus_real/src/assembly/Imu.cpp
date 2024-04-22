#include "orthrus_real/assembly/Imu.hpp"

#include <inttypes.h>
#include <cstdio>

namespace orthrus_real
{
    void Imu::Correction(double standard_yaw)
    {
        unified_gyro_ = gyro_*correction_matrix_;

        GetEuler(unified_gyro_);

        Eigen::Vector3d eulerAngles(euler_(ROLL), euler_(PITCH), standard_yaw);

        standard_gyro_ = Eigen::Quaterniond(Eigen::AngleAxisd(eulerAngles.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(eulerAngles.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(eulerAngles.x(), Eigen::Vector3d::UnitX()));
    }

    void Imu::GetEuler(Eigen::Quaterniond input_quaterniond)
    {
        euler_ = input_quaterniond.toRotationMatrix().eulerAngles(ROLL, PITCH, YAW);

        //yaw = euler(0);
        //pitch = euler(1);
        //roll = euler(2);
    }

    void Imu::RotatingCoordinates(double angle1, Eigen::Vector3d axis1,double angle2, Eigen::Vector3d axis2)
    {
        Eigen::AngleAxisd rotation1(angle1, axis1); // 绕y轴旋转90度
        Eigen::Quaterniond q1(rotation1);

        Eigen::AngleAxisd rotation2(angle2,axis2);
        Eigen::Quaterniond q2(rotation2);

        this->unified_gyro_ = this->gyro_ * q2 * q1;
    }

    void Imu::CorrectionMatrixSet(double angle1, Eigen::Vector3d axis1,double angle2, Eigen::Vector3d axis2)
    {
        Eigen::AngleAxisd rotation1(angle1, axis1);
        Eigen::Quaterniond q1(rotation1);

        Eigen::AngleAxisd rotation2(angle2,axis2);
        Eigen::Quaterniond q2(rotation2);

        correction_matrix_ = q2 * q1;
    }


    void Imu::IfUseMag(bool flag, can_pack can)
    {
        if (flag)
        {
            can.StdId = 0x11;
            can.DLC = 0x04;
            can.Data[0] = 0;
            can.Data[1] = 0;
            can.Data[2] = 0;
            can.Data[3] = 1;
        }
        else
        {
            can.StdId = 0x11;
            can.DLC = 0x04;
            can.Data[0] = 0;
            can.Data[1] = 0;
            can.Data[2] = 0;
            can.Data[3] = 0;
        }
    }

    void Imu::Init(uint8_t device_id)
    {
        device_id_ = device_id;
    }

    void Imu::Analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->can.StdId == device_id_ + 1)
        {
            memcpy(data.uint_data, &(pack->can.Data[0]), 4);
            gyro_.w() = data.f_data;

            memcpy(data.uint_data, &(pack->can.Data[4]), 4);
            gyro_.x() = data.f_data;
        }
        if (pack->can.StdId == device_id_ + 2)
        {
            memcpy(data.uint_data, &(pack->can.Data[0]), 4);
            gyro_.y() = data.f_data;

            memcpy(data.uint_data, &(pack->can.Data[4]), 4);
            gyro_.z() = data.f_data;
        }
    }
}