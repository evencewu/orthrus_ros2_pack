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
        void init(uint8_t leg_id, uint8_t motor_id);
        void analyze(Ecat_Inputs_Pack *pack);
        void SetOutput(Ecat_Outputs_Pack *pack, float k_p, float k_d, float W, float T, float Pos, int Mode);

        float T_ = 0;
        float Pos_ = 0;
        float W_ = 0;
        float Acc_ = 0;

        float RealPosition_ = 0;

    private:

        int leg_id_;
        int motor_id_;
    };
}