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
            data.uint_data[0] = pack->can[can_id].Data[0];
            data.uint_data[1] = pack->can[can_id].Data[1];
            data.uint_data[2] = pack->can[can_id].Data[2];
            data.uint_data[3] = pack->can[can_id].Data[3];

            Gyro[0] = data.f_data;

            data.uint_data[0] = pack->can[can_id].Data[4];
            data.uint_data[1] = pack->can[can_id].Data[5];
            data.uint_data[2] = pack->can[can_id].Data[6];
            data.uint_data[3] = pack->can[can_id].Data[7];

            Gyro[1] = data.f_data;
        }
        if (pack->can[can_id].StdId == device_id + 2)
        {
            data.uint_data[0] = pack->can[can_id].Data[0];
            data.uint_data[1] = pack->can[can_id].Data[1];
            data.uint_data[2] = pack->can[can_id].Data[2];
            data.uint_data[3] = pack->can[can_id].Data[3];

            Gyro[2] = data.f_data;
        }
    }
}