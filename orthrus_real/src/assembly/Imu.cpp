#include "orthrus_real/assembly/Imu.hpp"

#include <inttypes.h>
#include <cstdio>

namespace orthrus_real
{
    void Imu::Init(uint8_t can_id, uint8_t device_id)
    {
        can_id_ = can_id;
        device_id_ = device_id;
    }

    void Imu::Analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->can[can_id_].StdId == device_id_ + 1)
        {
            memcpy(data.uint_data, &(pack->can[can_id_].Data[0]), 4);
            gyro_.w() = data.f_data;

            memcpy(data.uint_data, &(pack->can[can_id_].Data[4]), 4);
            gyro_.x() = data.f_data;
        }
        if (pack->can[can_id_].StdId == device_id_ + 2)
        {
            memcpy(data.uint_data, &(pack->can[can_id_].Data[0]), 4);
            gyro_.y() = data.f_data;

            memcpy(data.uint_data, &(pack->can[can_id_].Data[4]), 4);
            gyro_.z() = data.f_data;
        }
    }
}