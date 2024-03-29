#pragma once

#include <inttypes.h>

#include "orthrus_real/ethercat/ecat_typedef.hpp"

namespace orthrus_real
{
    class Angle
    {
    public:
        void init(uint8_t can_id, uint8_t imu_id);
        void analyze(Ecat_Inputs_Pack *pack);

        float Pos; //角度

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
