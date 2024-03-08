#include "orthrus_real/orthrus_leg.hpp"

namespace orthrus_real
{
    void Leg::analyze(Ecat_Inputs_Pack *pack)
    {
        imu.analyze(pack);
        angle.analyze(pack);
    }

    void Leg::init(uint8_t can_id, uint8_t imu_id, uint8_t usart_id)
    {
        imu.init(can_id, imu_id);
        angle.init(can_id, imu_id);
    }
}