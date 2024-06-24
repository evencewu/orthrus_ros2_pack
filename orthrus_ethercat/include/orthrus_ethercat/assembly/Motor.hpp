#pragma once

#include "orthrus_ethercat/ethercat/TypeDef.hpp"

#include <inttypes.h>
#include <cstdio>
#include <string.h> 

#define RB 0
#define LB 1

#define RF 2
#define LF 3

namespace orthrus_ethercat
{
    class Motor
    {
    public:
        void Init(uint8_t leg_id, uint8_t motor_id);
        void Analyze(Ecat_Inputs_Pack *pack);
        void SetOutput(Ecat_Outputs_Pack *pack, float k_p, float k_d, float W, float T, float Pos, int Mode);

        float T_ = 0;
        float Pos_ = 0;
        float W_ = 0;
        float Acc_ = 0;
        int temp_ = 0;
        int error_ = 0;

        float RealPosition_ = 0;

    private:

        int leg_id_;
        int motor_id_;
    };
}