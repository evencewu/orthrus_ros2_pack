#include "orthrus_real/assembly/Encoder.hpp"

#include <cstring>

namespace orthrus_real
{
    void Encoder::Init(uint8_t can_id, uint8_t device_id)
    {
        this->can_id = can_id;
        this->device_id = device_id;
    }

    void Encoder::Analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->can[can_id].StdId == device_id + 2)
        {
            memcpy(data.uint_data, &(pack->can[can_id].Data[4]), 4);

            Pos = data.f_data;
        }
    }
}