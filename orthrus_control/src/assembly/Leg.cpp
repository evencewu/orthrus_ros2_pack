#include "orthrus_control/assembly/Leg.hpp"

namespace orthrus_control
{
    void Leg::Analyze(Ecat_Inputs_Pack *pack)
    {
        imu.Analyze(pack);
        angle.Analyze(pack);
        motor[0].Analyze(pack);
        motor[1].Analyze(pack);
        motor[2].Analyze(pack);
    }

    void Leg::Init(uint8_t imu_id, uint8_t leg_id, int slave_num)
    {
        imu.Init(imu_id);
        angle.Init(imu_id);
        motor[0].Init(leg_id, 0);
        motor[1].Init(leg_id, 1);
        motor[2].Init(leg_id, 2);
        slave_num_ = slave_num;
    }
}