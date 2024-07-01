#include "orthrus_control/OrthrusInterface.hpp"

namespace orthrus_control
{
    void OrthrusSystemHardware::SafeStop()
    {
        Ethercat.packet_tx[0].power = 0x00;
        Ethercat.packet_tx[1].power = 0x00;

        for (int i = 0; i <= 20; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                assembly_->leg[j / 3].motor[j % 3].SetOutput(&Ethercat.packet_tx[assembly_->leg[j / 3].slave_num_], 0, 0, 0, 0, 0, 0);
                Ethercat.EcatSyncMsg();
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "motor stop!");
    }

    // for debug
    void OrthrusSystemHardware::Log()
    {
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\n motor1 %lf %lf %lf %lf \n motor2 %lf %lf %lf %lf ",
        //             body_imu.euler_(PITCH),
        //             body_imu.euler_(PITCH),
        //             body_imu.euler_(ROLL),
        //             body_imu.euler_(ROLL),
        //             leg[0].imu.euler_(PITCH),
        //             leg[0].imu.euler_(PITCH),
        //             leg[0].imu.euler_(ROLL),
        //             leg[0].imu.euler_(ROLL));

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\n motor1 %lf %lf %lf %lf \n motor2 %lf %lf %lf %lf ",
        //             body_imu.euler_(PITCH) - leg[0].imu.euler_(ROLL),
        //             body_imu.euler_(PITCH) - leg[1].imu.euler_(ROLL),
        //             body_imu.euler_(PITCH) - leg[2].imu.euler_(ROLL),
        //             body_imu.euler_(PITCH) - leg[3].imu.euler_(ROLL),
        //             body_imu.euler_(ROLL) - leg[0].imu.euler_(PITCH),
        //             body_imu.euler_(ROLL) - leg[1].imu.euler_(PITCH),
        //             body_imu.euler_(ROLL) - leg[2].imu.euler_(PITCH),
        //             body_imu.euler_(ROLL) - leg[3].imu.euler_(PITCH));

        //RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu5 %lf %lf %lf  \033[0m", assembly_->body_imu.standard_acc_[0], assembly_->body_imu.standard_acc_[1], assembly_->body_imu.standard_acc_[2]);
        //RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu5 %lf %lf %lf  \033[0m", assembly_->body_imu.standard_angle_speed_[0], assembly_->body_imu.standard_angle_speed_[1], assembly_->body_imu.standard_angle_speed_[2]);
        //RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu5 %lf %lf %lf %lf \033[0m", assembly_->body_imu.unified_gyro_.w(), assembly_->body_imu.unified_gyro_.x(), assembly_->body_imu.unified_gyro_.y(), assembly_->body_imu.unified_gyro_.z());
        //RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu1 %lf %lf %lf %lf \033[0m", assembly_->leg[0].imu.gyro_.w(), assembly_->leg[0].imu.gyro_.x(), assembly_->leg[0].imu.gyro_.y(), assembly_->leg[0].imu.gyro_.z());
        //RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu2 %lf %lf %lf %lf \033[0m", assembly_->leg[1].imu.gyro_.w(), assembly_->leg[1].imu.gyro_.x(), assembly_->leg[1].imu.gyro_.y(), assembly_->leg[1].imu.gyro_.z());
        //RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu3 %lf %lf %lf %lf \033[0m", assembly_->leg[2].imu.gyro_.w(), assembly_->leg[2].imu.gyro_.x(), assembly_->leg[2].imu.gyro_.y(), assembly_->leg[2].imu.gyro_.z());
        //RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu4 %lf %lf %lf %lf \033[0m", assembly_->leg[3].imu.gyro_.w(), assembly_->leg[3].imu.gyro_.x(), assembly_->leg[3].imu.gyro_.y(), assembly_->leg[3].imu.gyro_.z());

        // all imu

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[0].motor[0].Pos_, leg[0].motor[1].Pos_, leg[0].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[1].motor[0].Pos_, leg[1].motor[1].Pos_, leg[1].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[2].motor[0].Pos_, leg[2].motor[1].Pos_, leg[2].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[3].motor[0].Pos_, leg[3].motor[1].Pos_, leg[3].motor[2].Pos_);

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[0].motor[0].Pos_, leg[0].motor[1].Pos_, leg[0].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[1].motor[0].Pos_, leg[1].motor[1].Pos_, leg[1].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[2].motor[0].Pos_, leg[2].motor[1].Pos_, leg[2].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[3].motor[0].Pos_, leg[3].motor[1].Pos_, leg[3].motor[2].Pos_);

        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "-----------------------------------------------");
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[0].motor[0].Pos_, leg[0].motor[1].Pos_, leg[0].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[1].motor[0].Pos_, leg[1].motor[1].Pos_, leg[1].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[2].motor[0].Pos_, leg[2].motor[1].Pos_, leg[2].motor[2].Pos_);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", leg[3].motor[0].Pos_, leg[3].motor[1].Pos_, leg[3].motor[2].Pos_);

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", command_effort[0], command_effort[1], command_effort[2]);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", command_effort[3], command_effort[4], command_effort[5]);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", command_effort[6], command_effort[7], command_effort[8]);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", command_effort[9], command_effort[10], command_effort[11]);

        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", dealta_real_position_[0][0], dealta_real_position_[0][1], dealta_real_position_[0][2]);
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", dealta_real_position_[1][0], dealta_real_position_[1][1], dealta_real_position_[1][2]);
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", dealta_real_position_[2][0], dealta_real_position_[2][1], dealta_real_position_[2][2]);
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", dealta_real_position_[3][0], dealta_real_position_[3][1], dealta_real_position_[3][2]);

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf %lf\033[0m", leg[0].angle.Pos_, leg[1].angle.Pos_, leg[2].angle.Pos_, leg[3].angle.Pos_);

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m stdid %d %d \033[0m", Ethercat.packet_rx[0].can.StdId, Ethercat.packet_rx[1].can.StdId);
    }
}