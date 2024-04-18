#include "orthrus_real/assembly/Imu.hpp"

#include <inttypes.h>
#include <cstdio>

namespace orthrus_real
{
    void Imu::get_angle(double standard_yaw)
    {
        // 从旋转矩阵中提取欧拉角（这里使用Z-Y-X顺序）
        Eigen::Vector3d euler = unified_gyro_.toRotationMatrix().eulerAngles(2, 1, 0);

        yaw = euler(0);
        pitch = euler(1);
        roll = euler(2);

        // yaw = standard_yaw;
        // yaw = M_PI/2;

        Eigen::Vector3d eulerAngles(roll, pitch, standard_yaw);

        standard_gyro_ = Eigen::Quaterniond(Eigen::AngleAxisd(eulerAngles.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(eulerAngles.y(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(eulerAngles.x(), Eigen::Vector3d::UnitX()));
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

    void Imu::Init(uint8_t can_id, uint8_t device_id)
    {
        can_id_ = can_id;
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