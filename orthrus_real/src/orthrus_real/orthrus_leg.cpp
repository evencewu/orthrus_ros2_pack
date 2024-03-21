#include "orthrus_real/orthrus_leg.hpp"

namespace orthrus_real
{
    void Leg::analyze(Ecat_Inputs_Pack *pack)
    {
        imu.analyze(pack);
        angle.analyze(pack);
        motor[0].analyze(pack);
        motor[1].analyze(pack);
        motor[2].analyze(pack);
    }

    void Leg::init(uint8_t can_id, uint8_t imu_id, uint8_t usart_id)
    {
        imu.init(can_id, imu_id);
        angle.init(can_id, imu_id);
        motor[0].init(can_id,usart_id*3);
        motor[1].init(can_id,usart_id*3+1);
        motor[2].init(can_id,usart_id*3+2);
    }
}