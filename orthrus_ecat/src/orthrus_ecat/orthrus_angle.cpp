#include "orthrus_ecat/orthrus_angle.hpp"

namespace orthrus_ecat
{
    Angle::Angle(uint8_t _can_id, int _device_id)
    {
        can_id = _can_id;
        device_id = _device_id;
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
