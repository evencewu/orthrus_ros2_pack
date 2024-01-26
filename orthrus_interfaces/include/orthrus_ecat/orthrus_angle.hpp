#pragma once

#include <inttypes.h>

#include "orthrus_ecat/ecat_typedef.hpp"

namespace orthrus_ecat
{
    class Angle
    {
    public:
        Angle(uint8_t _can_id,int _device_id);
        
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
