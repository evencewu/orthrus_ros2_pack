#include "orthrus_control/assembly/Imu.hpp"

#include <inttypes.h>
#include <cstdio>

namespace orthrus_control
{
    void Imu::Init(uint8_t device_id)
    {
        device_id_ = device_id;
        if_with_acc_ = false;
    }

    void Imu::Init(uint8_t device_id, int slave_num)
    {
        device_id_ = device_id;
        if_with_acc_ = true;
        slave_num_ = slave_num;
    }

    void Imu::Correction(double standard_yaw)
    {
        original_gyro_.normalize();

        unified_gyro_ = original_gyro_ * correction_matrix_;

        Eigen::Matrix3d rotation_matrix = correction_matrix_.toRotationMatrix();

        GetEuler(unified_gyro_);

        Eigen::Vector3d eulerAngles(euler_(ROLL), euler_(PITCH), standard_yaw);

        standard_acc_ = rotation_matrix.transpose() * original_acc_; // 
        standard_angle_speed_ = rotation_matrix.transpose() * original_angle_speed_;//correction_matrix_ * 
        standard_gyro_ = Eigen::Quaterniond(Eigen::AngleAxisd(eulerAngles.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(eulerAngles.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(eulerAngles.x(), Eigen::Vector3d::UnitX()));
    }

    void Imu::GetEuler(Eigen::Quaterniond input_quaterniond)
    {
        euler_ = input_quaterniond.toRotationMatrix().eulerAngles(ROLL, PITCH, YAW);
    }

    void Imu::CorrectionMatrixSet(double angle1, Eigen::Vector3d axis1, double angle2, Eigen::Vector3d axis2)
    {
        Eigen::AngleAxisd rotation1(angle1, axis1);
        Eigen::Quaterniond q1(rotation1);

        Eigen::AngleAxisd rotation2(angle2, axis2);
        Eigen::Quaterniond q2(rotation2);

        correction_matrix_ = q2 * q1;
    }

    void Imu::IfUseMag(bool flag, can_pack can)
    {
        if (flag)
        {
            can.StdId = 0x100;
            can.DLC = 0x04;
            can.Data[0] = 0;
            can.Data[1] = 0;
            can.Data[2] = 0;
            can.Data[3] = 1;
        }
        else
        {
            can.StdId = 0x100;
            can.DLC = 0x04;
            can.Data[0] = 0;
            can.Data[1] = 0;
            can.Data[2] = 0;
            can.Data[3] = 0;
        }
    }

    void Imu::Analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->can.StdId == device_id_ + 1)
        {
            memcpy(data.uint_data, &(pack->can.Data[0]), 4);
            original_gyro_.w() = data.f_data;

            memcpy(data.uint_data, &(pack->can.Data[4]), 4);
            original_gyro_.x() = data.f_data;
        }
        if (pack->can.StdId == device_id_ + 2)
        {
            memcpy(data.uint_data, &(pack->can.Data[0]), 4);
            original_gyro_.y() = data.f_data;

            memcpy(data.uint_data, &(pack->can.Data[4]), 4);
            original_gyro_.z() = data.f_data;
        }

        if (if_with_acc_)
        {
            if (pack->can.StdId == device_id_ + 4)
            {
                memcpy(data.uint_data, &(pack->can.Data[0]), 4);
                original_acc_[0] = data.f_data;

                memcpy(data.uint_data, &(pack->can.Data[4]), 4);
                original_acc_[1] = data.f_data;
            }

            if (pack->can.StdId == device_id_ + 5)
            {
                memcpy(data.uint_data, &(pack->can.Data[0]), 4);
                original_acc_[2] = data.f_data;

                memcpy(data.uint_data, &(pack->can.Data[4]), 4);
                original_angle_speed_[0] = data.f_data;
            }

            if (pack->can.StdId == device_id_ + 6)
            {
                memcpy(data.uint_data, &(pack->can.Data[0]), 4);
                original_angle_speed_[1] = data.f_data;

                memcpy(data.uint_data, &(pack->can.Data[4]), 4);
                original_angle_speed_[2] = data.f_data;
            }
        }
    }
}