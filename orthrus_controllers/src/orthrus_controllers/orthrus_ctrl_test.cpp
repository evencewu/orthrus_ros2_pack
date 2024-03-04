#include "orthrus_controllers/orthrus_ctrl_test.hpp"

namespace othrus_ctrl
{
    orthrus_interfaces::msg::CtrlCmd PositonCtrl::StandUp()
    {
        ctrl_cmd_msg_.motor_cmd[0].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[0].target_p = 0;
        ctrl_cmd_msg_.motor_cmd[0].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[0].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[0].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[1].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[1].target_p = PI/3;
        ctrl_cmd_msg_.motor_cmd[1].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[1].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[1].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[2].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[2].target_p = -PI/2;
        ctrl_cmd_msg_.motor_cmd[2].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[2].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[2].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[3].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[3].target_p = 0;
        ctrl_cmd_msg_.motor_cmd[3].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[3].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[3].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[4].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[4].target_p = -PI/3;
        ctrl_cmd_msg_.motor_cmd[4].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[4].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[4].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[5].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[5].target_p = PI/2;
        ctrl_cmd_msg_.motor_cmd[5].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[5].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[5].torqe = 0; 

        ctrl_cmd_msg_.motor_cmd[6].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[6].target_p = 0;
        ctrl_cmd_msg_.motor_cmd[6].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[6].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[6].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[7].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[7].target_p = PI/3;
        ctrl_cmd_msg_.motor_cmd[7].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[7].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[7].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[8].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[8].target_p = -PI/2;
        ctrl_cmd_msg_.motor_cmd[8].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[8].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[8].torqe = 0; 

        ctrl_cmd_msg_.motor_cmd[9].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[9].target_p = 0;
        ctrl_cmd_msg_.motor_cmd[9].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[9].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[9].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[10].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[10].target_p = -PI/3;
        ctrl_cmd_msg_.motor_cmd[10].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[10].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[10].torqe = 0;

        ctrl_cmd_msg_.motor_cmd[11].k_p = 30;
        ctrl_cmd_msg_.motor_cmd[11].target_p = PI/2;
        ctrl_cmd_msg_.motor_cmd[11].k_d = 5;
        ctrl_cmd_msg_.motor_cmd[11].target_d = 0;
        ctrl_cmd_msg_.motor_cmd[11].torqe = 0; 

        return ctrl_cmd_msg_;
    }
}