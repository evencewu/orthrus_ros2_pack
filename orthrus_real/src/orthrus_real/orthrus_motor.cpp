#include "orthrus_real/orthrus_motor.hpp"

namespace orthrus_real
{
    void Motor::init(uint8_t leg_id, uint8_t motor_id)
    {
        leg_id_ = leg_id;
        motor_id_ = motor_id;
    }

    void Motor::analyze(Ecat_Inputs_Pack *pack)
    {
        Pos_ = pack->motor[leg_id_][motor_id_].Pos;
        T_ = pack->motor[leg_id_][motor_id_].T;
        W_ = pack->motor[leg_id_][motor_id_].W;
        Acc_ = pack->motor[leg_id_][motor_id_].Acc;
        temp_ = pack->motor[leg_id_][motor_id_].temp;
        error_ = pack->motor[leg_id_][motor_id_].error;
    }

    void Motor::SetOutput(Ecat_Outputs_Pack *pack, float k_p, float k_d, float W, float T, float Pos, int Mode)
    {
        pack->motor[leg_id_][motor_id_].mode = Mode;
        pack->motor[leg_id_][motor_id_].kp = k_p;
        pack->motor[leg_id_][motor_id_].kd = k_d;
        pack->motor[leg_id_][motor_id_].w = W;
        pack->motor[leg_id_][motor_id_].t = T;
        pack->motor[leg_id_][motor_id_].pos = Pos;
    }
}