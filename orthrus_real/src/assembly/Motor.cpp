#include "orthrus_real/assembly/Motor.hpp"

namespace orthrus_real
{
    void Motor::Init(uint8_t leg_id, uint8_t motor_id)
    {
        leg_id_ = leg_id;
        motor_id_ = motor_id;
    }

    void Motor::Analyze(Ecat_Inputs_Pack *pack)
    {
        if (pack->motor_id == (leg_id_%2) * 3 + motor_id_)
        {
            Pos_ = pack->motor.Pos;
            T_ = pack->motor.T;
            W_ = pack->motor.W;
            Acc_ = pack->motor.Acc;
            temp_ = pack->motor.temp;
            error_ = pack->motor.error;
        }
    }

    void Motor::SetOutput(Ecat_Outputs_Pack *pack, float k_p, float k_d, float W, float T, float Pos, int Mode)
    {
        pack->motor_id = (leg_id_%2) * 3 + motor_id_;
        pack->motor.mode = Mode;
        pack->motor.kp = k_p;
        pack->motor.kd = k_d;
        pack->motor.w = W;
        pack->motor.t = T;
        pack->motor.pos = Pos;
    }
}