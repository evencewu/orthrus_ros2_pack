#pragma once

#include <inttypes.h>
#include <cstdio>

#include "orthrus_real/ecat_typedef.hpp"

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

    //plan C
    class MotorCan
    {
    public:

        void init(uint8_t can_id, uint8_t device_id,uint8_t motor_id);
        void analyze(Ecat_Inputs_Pack *pack);
        void SetOutput(Ecat_Outputs_Pack *pack, int can_pack, float k_p, float k_d, float W, float T, float Pos, uint8_t Mode);

        float T_ = 0;
        float Pos_ = 0;
        float W_ = 0;
        float Acc_ = 0;

        //struct orthrus_motor
        //{
        //    float k_p_;
        //    float k_d_;
        //    float W_;
        //    float T_;
        //    float Pos_;
        //    uint16_t Mode_;
        //    uint16_t CR_;
        //};
    private:
        union
        {
            float f_data;
            uint8_t uint_data[4];
        }data;

        uint8_t can_id;
        uint8_t leg_id;
        uint8_t motor_id;
    };
}