#pragma once

#include <inttypes.h>

#include "orthrus_control/ethercat/TypeDef.hpp"

namespace orthrus_control
{
    class Encoder
    {
    public:
        void Init(uint8_t imu_id);
        void Analyze(Ecat_Inputs_Pack *pack);

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