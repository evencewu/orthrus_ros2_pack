#include "orthrus_controllers/orthrus_wbc.hpp"

namespace orthrus_ctrl
{
    OrthrusWbc::OrthrusWbc()
    {

    }

    orthrus_interfaces::msg::OrthrusJointControl OrthrusWbc::StandUp()
    {
        orthrus_joint_control_msg_.motor_cmd[0].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[0].target_p = 0;
        orthrus_joint_control_msg_.motor_cmd[0].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[0].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[0].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[1].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[1].target_p = PI/3;
        orthrus_joint_control_msg_.motor_cmd[1].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[1].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[1].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[2].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[2].target_p = -PI/2;
        orthrus_joint_control_msg_.motor_cmd[2].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[2].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[2].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[3].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[3].target_p = 0;
        orthrus_joint_control_msg_.motor_cmd[3].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[3].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[3].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[4].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[4].target_p = -PI/3;
        orthrus_joint_control_msg_.motor_cmd[4].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[4].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[4].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[5].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[5].target_p = PI/2;
        orthrus_joint_control_msg_.motor_cmd[5].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[5].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[5].torqe = 0; 

        orthrus_joint_control_msg_.motor_cmd[6].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[6].target_p = 0;
        orthrus_joint_control_msg_.motor_cmd[6].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[6].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[6].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[7].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[7].target_p = PI/3;
        orthrus_joint_control_msg_.motor_cmd[7].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[7].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[7].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[8].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[8].target_p = -PI/2;
        orthrus_joint_control_msg_.motor_cmd[8].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[8].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[8].torqe = 0; 

        orthrus_joint_control_msg_.motor_cmd[9].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[9].target_p = 0;
        orthrus_joint_control_msg_.motor_cmd[9].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[9].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[9].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[10].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[10].target_p = -PI/3;
        orthrus_joint_control_msg_.motor_cmd[10].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[10].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[10].torqe = 0;

        orthrus_joint_control_msg_.motor_cmd[11].k_p = 30;
        orthrus_joint_control_msg_.motor_cmd[11].target_p = PI/2;
        orthrus_joint_control_msg_.motor_cmd[11].k_d = 5;
        orthrus_joint_control_msg_.motor_cmd[11].target_d = 0;
        orthrus_joint_control_msg_.motor_cmd[11].torqe = 0; 

        return orthrus_joint_control_msg_;
    }


}
