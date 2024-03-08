#include "orthrus_real/orthrus_angle.hpp"

namespace orthrus_real
{
    void Angle::init(uint8_t can_id, uint8_t device_id)
    {
        this->can_id = can_id;
        this->device_id = can_id;
    }

    void Angle::analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->can[can_id].StdId == device_id + 2)
        {
            data.uint_data[0] = pack->can[can_id].Data[4];
            data.uint_data[1] = pack->can[can_id].Data[5];
            data.uint_data[2] = pack->can[can_id].Data[6];
            data.uint_data[3] = pack->can[can_id].Data[7];

            Pos = data.f_data;
        }
    }
}
