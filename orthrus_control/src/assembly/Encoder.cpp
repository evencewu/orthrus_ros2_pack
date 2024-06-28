#include "orthrus_control/assembly/Encoder.hpp"

#include <cstring>

namespace orthrus_control
{
    void Encoder::Init(uint8_t device_id)
    {
        this->device_id = device_id;
    }

    void Encoder::Analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->can.StdId == device_id + 3)
        {
            memcpy(data.uint_data, &(pack->can.Data[0]), 4);

            Pos_ = data.f_data;
        }
    }
}