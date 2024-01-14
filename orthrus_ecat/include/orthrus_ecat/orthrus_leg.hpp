#pragma once

#include <inttypes.h>

#include "orthrus_ecat/ecat_typedef.hpp"

#include "orthrus_ecat/orthrus_imu.hpp"
#include "orthrus_ecat/orthrus_angle.hpp"

namespace orthrus_ecat
{
    class Leg
    {
    public:
        Leg(uint8_t _can_id, uint8_t _imu_id, uint8_t _usart_id);

        void analyze(Ecat_Inputs_Pack *pack);

        Imu imu = Imu(1,imu_id);
        Angle angle = Angle(1,imu_id);
    private:
        uint8_t can_id;
        uint8_t imu_id;
        uint8_t usart_id;
    };
}