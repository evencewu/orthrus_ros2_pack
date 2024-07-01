#include "orthrus_controller/safecode/SafeCode.hpp"

namespace orthrus_controller
{
    void SafeCode::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
    }

    bool SafeCode::PositionCheck()
    {
        bool flag = true;
        int motor_max_min_pos[3][2];

        motor_max_min_pos[0][0] = M_PI / 3;
        motor_max_min_pos[0][1] = -M_PI / 3;
        motor_max_min_pos[1][0] = M_PI * 2 / 3;
        motor_max_min_pos[1][1] = -M_PI / 3;
        motor_max_min_pos[2][0] = -M_PI / 6;
        motor_max_min_pos[2][1] = -(M_PI * 7) / 12;

        for (int i = 0; i < 12; i++)
        {

            if (i < 6)
            {
                if (orthrus_interfaces_->robot_state.joint.position[i] > -motor_max_min_pos[i % 3][1])
                {

                    if (orthrus_interfaces_->robot_cmd.effort[i] > 0)
                    {
                        flag = false;
                        orthrus_interfaces_->robot_cmd.effort[i] = 0;
                    }
                }

                if (orthrus_interfaces_->robot_state.joint.position[i] < -motor_max_min_pos[i % 3][0])
                {
                    if (orthrus_interfaces_->robot_cmd.effort[i] < 0)
                    {
                        flag = false;
                        orthrus_interfaces_->robot_cmd.effort[i] = 0;
                    }
                }
            }
            else
            {
                if (orthrus_interfaces_->robot_state.joint.position[i] > motor_max_min_pos[i % 3][0])
                {
                    if (orthrus_interfaces_->robot_cmd.effort[i] > 0)
                    {
                        flag = false;
                        orthrus_interfaces_->robot_cmd.effort[i] = 0;
                    }
                }

                if (orthrus_interfaces_->robot_state.joint.position[i] < motor_max_min_pos[i % 3][1])
                {
                    if (orthrus_interfaces_->robot_cmd.effort[i] < 0)
                    {
                        flag = false;
                        orthrus_interfaces_->robot_cmd.effort[i] = 0;
                    }
                }
            }
        }

        return flag;
    }

}