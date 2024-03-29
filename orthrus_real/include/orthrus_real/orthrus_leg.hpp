#pragma once

#include <inttypes.h>

#include "orthrus_real/ethercat/ecat_typedef.hpp"
#include "orthrus_real/orthrus_imu.hpp"
#include "orthrus_real/orthrus_angle.hpp"
#include "orthrus_real/orthrus_motor.hpp"

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
        void init(uint8_t can_id, uint8_t imu_id, uint8_t usart_id);
        void analyze(Ecat_Inputs_Pack *pack);

        Imu imu;
        Angle angle;
        MotorCan motor[3];
    };
}