#pragma once

#include <inttypes.h>

#include "orthrus_real/ecat_typedef.hpp"

#define IMU1 0
#define IMU2 2
#define IMU3 4
#define IMU4 6
#define IMU5 8

namespace orthrus_real
{
    class Imu
    {
    public:
        void init(uint8_t can_id, uint8_t device_id);
        void analyze(Ecat_Inputs_Pack *pack);

        float Gyro[3]; //角度
    private:
        union
        {
            float f_data;
            uint8_t uint_data[4];
        }data;
        
        uint8_t can_id;
        uint8_t device_id;
    };
}