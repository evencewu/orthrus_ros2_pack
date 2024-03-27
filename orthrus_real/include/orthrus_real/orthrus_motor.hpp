#pragma once

#include <inttypes.h>
#include <cstdio>
#include <string.h> 

#include "orthrus_real/ethercat/ecat_typedef.hpp"

#define RB 0
#define RF 1
#define LB 2
#define LF 3

namespace orthrus_real
{
    class Motor
    {
    public:
        Motor(uint8_t _motor_id);

    private:
    };

    // plan C
    class MotorCan
    {
    public:
        void init(uint8_t can_id, uint8_t device_id, uint8_t motor_id);
        void analyze(Ecat_Inputs_Pack *pack);
        void SetOutput(Ecat_Outputs_Pack *pack, int can_pack, float k_p, float k_d, float W, float T, float Pos, uint8_t Mode);

        float T_ = 0;
        float Pos_ = 0;
        float W_ = 0;
        float Acc_ = 0;

        float RealPosition_ = 0;

    private:
        union
        {
            float f_data;
            uint8_t uint_data[4];
        } data;

        union
        {
            float f_data;
            uint32_t uint_data;
        } float_to_uint32_data;

        union
        {
            uint16_t f_data;
            uint8_t uint_data[2];
        } data16;

        uint8_t can_id;
        uint8_t leg_id;
        uint8_t motor_id;
    };
}