#include "orthrus_real/orthrus_imu.hpp"

#include <inttypes.h>
#include <cstdio>

namespace orthrus_real
{
    void Imu::init(uint8_t can_id, uint8_t device_id)
    {
        this->can_id = can_id;
        this->device_id = device_id;
    }

    void Imu::analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->can[can_id].StdId == device_id + 1)
        {
            memcpy(data.uint_data, &(pack->can[can_id].Data[0]),4);
            gyro_.w() = data.f_data;

            memcpy(data.uint_data, &(pack->can[can_id].Data[4]),4);
            gyro_.x() = data.f_data;
        }
        if (pack->can[can_id].StdId == device_id + 2)
        {
            memcpy(data.uint_data, &(pack->can[can_id].Data[0]),4);
            gyro_.y() = data.f_data;

            memcpy(data.uint_data, &(pack->can[can_id].Data[4]),4);
            gyro_.z() = data.f_data;
        }
    }
}