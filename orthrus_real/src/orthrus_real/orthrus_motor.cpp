#include "orthrus_real/orthrus_motor.hpp"


namespace orthrus_real
{
    void MotorCan::init(uint8_t can_id, uint8_t device_id, uint8_t motor_id)
    {
        this->can_id = can_id;
        this->leg_id = device_id;
        this->motor_id = motor_id;
    }

    void MotorCan::analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->can[can_id].StdId == leg_id * 6 + motor_id * 2 + 0x40)
        {
            memcpy(data.uint_data, &(pack->can[can_id].Data[0]),4);
            T_ = data.f_data;

            memcpy(data.uint_data, &(pack->can[can_id].Data[4]),4);
            Pos_ = data.f_data;
        }

        if (pack->can[can_id].StdId == leg_id * 6 + motor_id * 2 + 0x41)
        {
            memcpy(data.uint_data, &(pack->can[can_id].Data[0]),4);
            W_ = data.f_data;

            memcpy(data.uint_data, &(pack->can[can_id].Data[4]),4);
            Acc_ = data.f_data;
        }
    }

    void MotorCan::SetOutput(Ecat_Outputs_Pack *pack, int can_pack, float k_p, float k_d, float W, float T, float Pos, uint8_t Mode)
    {
        pack->can[can_id].DLC = 0x08;
        if (can_pack == 0)
        {
            pack->can[can_id].StdId = leg_id * 9 + motor_id * 3 + 0x20;

            data.f_data = k_p;

            pack->can[can_id].Data[0] = data.uint_data[0];
            pack->can[can_id].Data[1] = data.uint_data[1];
            pack->can[can_id].Data[2] = data.uint_data[2];
            pack->can[can_id].Data[3] = data.uint_data[3];

            data.f_data = k_d;

            pack->can[can_id].Data[4] = data.uint_data[0];
            pack->can[can_id].Data[5] = data.uint_data[1];
            pack->can[can_id].Data[6] = data.uint_data[2];
            pack->can[can_id].Data[7] = data.uint_data[3];
        }

        if (can_pack == 1)
        {
            pack->can[can_id].StdId = leg_id * 9 + motor_id * 3 + 0x21;

            data.f_data = W;

            pack->can[can_id].Data[0] = data.uint_data[0];
            pack->can[can_id].Data[1] = data.uint_data[1];
            pack->can[can_id].Data[2] = data.uint_data[2];
            pack->can[can_id].Data[3] = data.uint_data[3];

            data.f_data = T;

            pack->can[can_id].Data[4] = data.uint_data[0];
            pack->can[can_id].Data[5] = data.uint_data[1];
            pack->can[can_id].Data[6] = data.uint_data[2];
            pack->can[can_id].Data[7] = data.uint_data[3];
        }

        if (can_pack == 2)
        {
            pack->can[can_id].StdId = leg_id * 9 + motor_id * 3 + 0x22;

            data.f_data = Pos;

            pack->can[can_id].Data[0] = data.uint_data[0];
            pack->can[can_id].Data[1] = data.uint_data[1];
            pack->can[can_id].Data[2] = data.uint_data[2];
            pack->can[can_id].Data[3] = data.uint_data[3];

            pack->can[can_id].Data[4] = Mode;
            pack->can[can_id].Data[5] = 0;

            data16.f_data = 0;

            float_to_uint32_data.f_data = k_p;
            data16.f_data += float_to_uint32_data.uint_data;

            float_to_uint32_data.f_data = k_d;
            data16.f_data += float_to_uint32_data.uint_data;

            float_to_uint32_data.f_data = W;
            data16.f_data += float_to_uint32_data.uint_data;

            float_to_uint32_data.f_data = T;
            data16.f_data += float_to_uint32_data.uint_data;

            float_to_uint32_data.f_data = Pos;
            data16.f_data += float_to_uint32_data.uint_data;

            data16.f_data += Mode;

            pack->can[can_id].Data[6] = data16.uint_data[0];
            pack->can[can_id].Data[7] = data16.uint_data[1];
        }
    }
}