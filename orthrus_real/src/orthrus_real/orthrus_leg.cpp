#include "orthrus_real/orthrus_leg.hpp"

namespace orthrus_real
{
    Leg::Leg(uint8_t _can_id, uint8_t _imu_id, uint8_t _usart_id)
    {
        can_id = _can_id;
        imu_id = _imu_id;
        usart_id = _usart_id;

        imu = Imu(can_id,imu_id);
        angle = Angle(can_id,imu_id);
    }

    void Leg::analyze(Ecat_Inputs_Pack *pack)
    {
        imu.analyze(pack);
        angle.analyze(pack);
    }
}