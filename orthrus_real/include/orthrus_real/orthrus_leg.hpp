#pragma once

#include <inttypes.h>

#include "orthrus_real/ethercat/TypeDef.hpp"
#include "orthrus_real/assembly/Imu.hpp"
#include "orthrus_real/assembly/Encoder.hpp"
#include "orthrus_real/assembly/Motor.hpp"

#include <cstdio>

#define USART1 0
#define USART2 1
#define USART3 2
#define USART6 3

namespace orthrus_real
{
    class Leg
    {
    public:
        void Init(uint8_t imu_id, uint8_t usart_id);
        void Analyze(Ecat_Inputs_Pack *pack);

        Imu imu;
        Encoder angle;
        Motor motor[3];
    };
}