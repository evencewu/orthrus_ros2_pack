#include "orthrus_real/orthrus_leg.hpp"

namespace orthrus_real
{
    void Leg::Analyze(Ecat_Inputs_Pack *pack)
    {
        imu.Analyze(pack);
        angle.Analyze(pack);
        motor[0].Analyze(pack);
        motor[1].Analyze(pack);
        motor[2].Analyze(pack);
    }

    void Leg::Init(uint8_t can_id, uint8_t imu_id, uint8_t leg_id)
    {
        imu.Init(can_id, imu_id);
        angle.Init(can_id, imu_id);
        motor[0].Init(leg_id, 0);
        motor[1].Init(leg_id, 1);
        motor[2].Init(leg_id, 2);
    }
}