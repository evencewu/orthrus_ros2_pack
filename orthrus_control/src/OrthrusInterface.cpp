#include "orthrus_control/OrthrusInterface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
namespace orthrus_control
{
    hardware_interface::CallbackReturn OrthrusSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "OrthrusHardware on init");
        // hardware init
        leg[0].Init(IMU2, USART1);
        leg[1].Init(IMU1, USART2);
        leg[2].Init(IMU3, USART3);
        leg[3].Init(IMU4, USART6);
        body_imu.Init(IMU5, TRUE);
        // 错误检查

        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            // DiffBotSystem has exactly two states and one command interface on each joint
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("OrthrusHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("OrthruHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 3)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("OrthruHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("OrthruHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("OrthruHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("OrthruHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        for (const auto &gpio : info.gpios)
        {
            for (const auto &cmd_iface : gpio.command_interfaces)
            {
                gpio_commands_[cmd_iface.name] = std::numeric_limits<double>::quiet_NaN();
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "OrthrusHardware init finish");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> OrthrusSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_[i]));
        }

        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> OrthrusSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
        }

        for (auto &gpio_command : gpio_commands_)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                "flag", gpio_command.first, &gpio_command.second));
        }

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "motor stop!" );

        return command_interfaces;
    }

    hardware_interface::CallbackReturn OrthrusSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m Activating ...please wait...\033[0m");
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        // set some default values
        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_effort_[i] = 0;
                hw_commands_[i] = 0;
                gpio_commands_["enable_power"] = 0;
                gpio_commands_["calibration_position"] = 0;
                gpio_commands_["calibration_encoder"] = 0;
            }
        }

        subscriber_is_active_ = true;
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m Finish activate\033[0m");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OrthrusSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        Ethercat.EcatStop();

        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m Successfully deactivated! \033[0m");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type orthrus_control::OrthrusSystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        if (ethercat_prepare_flag_ == false)
        {
            time_last_ethercat_ = std::chrono::high_resolution_clock::now();
            char phy[] = "enp2s0";
            bool ecat_flag = false;
            ecat_flag = Ethercat.EcatStart(phy);
            if (ecat_flag)
            {
                bool sync_flag = false;

                sync_flag = Ethercat.EcatSyncMsg();
                if (sync_flag)
                {
                    ethercat_prepare_flag_ = true;
                }
            }
        }

        // ros2 control 交换数据

        hw_positions_[0] = -leg[1].motor[0].Pos_ / 9.1 - dealta_real_position_[1][0];
        hw_positions_[1] = leg[1].motor[1].Pos_ / 9.1 - dealta_real_position_[1][1];
        hw_positions_[2] = leg[1].motor[2].Pos_ / 9.1 + (30 * M_PI / 180 - theta1 * M_PI / 180) - dealta_real_position_[1][2]; //

        hw_positions_[3] = -leg[0].motor[0].Pos_ / 9.1 - dealta_real_position_[0][0];
        hw_positions_[4] = leg[0].motor[1].Pos_ / 9.1 - dealta_real_position_[0][1];
        hw_positions_[5] = leg[0].motor[2].Pos_ / 9.1 - (30 * M_PI / 180 - theta1 * M_PI / 180) - dealta_real_position_[0][2];

        hw_positions_[6] = -leg[3].motor[0].Pos_ / 9.1 - dealta_real_position_[3][0];
        hw_positions_[7] = leg[3].motor[1].Pos_ / 9.1 - dealta_real_position_[3][1];
        hw_positions_[8] = leg[3].motor[2].Pos_ / 9.1 + (30 * M_PI / 180 - theta1 * M_PI / 180) - dealta_real_position_[3][2];

        hw_positions_[9] = -leg[2].motor[0].Pos_ / 9.1 - dealta_real_position_[2][0];
        hw_positions_[10] = leg[2].motor[1].Pos_ / 9.1 - dealta_real_position_[2][1];
        hw_positions_[11] = leg[2].motor[2].Pos_ / 9.1 - (30 * M_PI / 180 - theta1 * M_PI / 180) - dealta_real_position_[2][2];

        // imu

        Eigen::Quaterniond q_relative = body_imu.gyro_ * body_imu.gyro_.inverse();

        Eigen::Matrix3d rotation_matrix = q_relative.toRotationMatrix();

        Eigen::Vector3d fix_angle_speed = rotation_matrix * body_imu.angle_speed_;

        hw_sensor_states_[4] = fix_angle_speed[0];
        hw_sensor_states_[5] = fix_angle_speed[1];
        hw_sensor_states_[6] = fix_angle_speed[2];

        Eigen::Vector3d fix_acc = rotation_matrix * body_imu.acc_;

        hw_sensor_states_[7] = fix_acc[0];
        hw_sensor_states_[8] = fix_acc[1];
        hw_sensor_states_[9] = fix_acc[2];

        hw_sensor_states_[3] = body_imu.unified_gyro_.w();
        hw_sensor_states_[0] = body_imu.unified_gyro_.x();
        hw_sensor_states_[1] = body_imu.unified_gyro_.y();
        hw_sensor_states_[2] = body_imu.unified_gyro_.z();

        ethercat_prepare_flag_ = Ethercat.EcatSyncMsg();

        UnifiedSensorData();

        time_now_ = std::chrono::high_resolution_clock::now();

        if (ethercat_prepare_flag_ != true)
        {
            std::chrono::duration<double> duration = time_now_ - time_last_ethercat_;
            if (duration.count() >= 4)
            {
                SafeStop();
                Ethercat.EcatStop();
            }
        }

        // Log();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type orthrus_control::OrthrusSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        command_effort[0] = hw_commands_[3];
        command_effort[1] = hw_commands_[4];
        command_effort[2] = hw_commands_[5];

        command_effort[3] = hw_commands_[0];
        command_effort[4] = hw_commands_[1];
        command_effort[5] = hw_commands_[2];

        command_effort[6] = hw_commands_[9];
        command_effort[7] = hw_commands_[10];
        command_effort[8] = hw_commands_[11];

        command_effort[9] = hw_commands_[6];
        command_effort[10] = hw_commands_[7];
        command_effort[11] = hw_commands_[8];

        if (motorcan_send_flag_ < 6)
        {
            leg[motorcan_send_flag_ / 3].motor[motorcan_send_flag_ % 3].SetOutput(&Ethercat.packet_tx[0], 0, 0, 0, command_effort[motorcan_send_flag_], 0, 10);
        }
        else
        {
            leg[motorcan_send_flag_ / 3].motor[motorcan_send_flag_ % 3].SetOutput(&Ethercat.packet_tx[1], 0, 0, 0, command_effort[motorcan_send_flag_], 0, 10);
        }

        motorcan_send_flag_++;
        if (motorcan_send_flag_ == 12)
        {
            motorcan_send_flag_ = 0;
        }

        if (gpio_commands_["enable_power"])
        {
            Ethercat.packet_tx[0].power = 0x01;
            Ethercat.packet_tx[1].power = 0x01;
        }
        else
        {
            Ethercat.packet_tx[0].power = 0x00;
            Ethercat.packet_tx[1].power = 0x00;
        }

        if (gpio_commands_["calibration_position"])
        {
            CalibrationPosition();
        }
        else
        {
        }

        // if (gpio_commands_["calibration_encoder"])
        //{
        //      StartCalibrationEncoderPosition();
        //  }
        // else
        //{
        //      StopCalibrationEncoderPosition();
        //  }

        leg[0].Analyze(&Ethercat.packet_rx[1]);
        leg[1].Analyze(&Ethercat.packet_rx[1]);
        leg[2].Analyze(&Ethercat.packet_rx[0]);
        leg[3].Analyze(&Ethercat.packet_rx[0]);
        body_imu.Analyze(&Ethercat.packet_rx[1]);

        return hardware_interface::return_type::OK;
    }

    void OrthrusSystemHardware::SafeStop()
    {
        Ethercat.packet_tx[0].power = 0x00;
        Ethercat.packet_tx[1].power = 0x00;

        for (int i = 0; i <= 20; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                if (j > 5)
                {
                    leg[j / 3].motor[j % 3].SetOutput(&Ethercat.packet_tx[1], 0, 0, 0, 0, 0, 0);
                    Ethercat.EcatSyncMsg();
                }
                else
                {
                    leg[j / 3].motor[j % 3].SetOutput(&Ethercat.packet_tx[0], 0, 0, 0, 0, 0, 0);
                    Ethercat.EcatSyncMsg();
                }
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

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu5 %lf %lf %lf  \033[0m", body_imu.acc_[0], body_imu.acc_[1], body_imu.acc_[2]);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu5 %lf %lf %lf  \033[0m", body_imu.angle_speed_[0], body_imu.angle_speed_[1], body_imu.angle_speed_[2]);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu5 %lf %lf %lf %lf \033[0m", body_imu.gyro_.w(), body_imu.gyro_.x(), body_imu.gyro_.y(), body_imu.gyro_.z());
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu5 %lf %lf %lf %lf \033[0m", body_imu.unified_gyro_.w(), body_imu.unified_gyro_.x(), body_imu.unified_gyro_.y(), body_imu.unified_gyro_.z());
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu1 %lf %lf %lf %lf \033[0m", leg[0].imu.gyro_.w(), leg[0].imu.gyro_.x(), leg[0].imu.gyro_.y(), leg[0].imu.gyro_.z());
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu2 %lf %lf %lf %lf \033[0m", leg[1].imu.gyro_.w(), leg[1].imu.gyro_.x(), leg[1].imu.gyro_.y(), leg[1].imu.gyro_.z());
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu3 %lf %lf %lf %lf \033[0m", leg[2].imu.gyro_.w(), leg[2].imu.gyro_.x(), leg[2].imu.gyro_.y(), leg[2].imu.gyro_.z());
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m imu4 %lf %lf %lf %lf \033[0m", leg[3].imu.gyro_.w(), leg[3].imu.gyro_.x(), leg[3].imu.gyro_.y(), leg[3].imu.gyro_.z());

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

        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", command_effort[0], command_effort[1], command_effort[2]);
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", command_effort[3], command_effort[4], command_effort[5]);
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", command_effort[6], command_effort[7], command_effort[8]);
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", command_effort[9], command_effort[10], command_effort[11]);

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", dealta_real_position_[0][0], dealta_real_position_[0][1], dealta_real_position_[0][2]);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", dealta_real_position_[1][0], dealta_real_position_[1][1], dealta_real_position_[1][2]);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", dealta_real_position_[2][0], dealta_real_position_[2][1], dealta_real_position_[2][2]);
        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf  \033[0m", dealta_real_position_[3][0], dealta_real_position_[3][1], dealta_real_position_[3][2]);

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m motor  %lf %lf %lf %lf\033[0m", leg[0].angle.Pos_, leg[1].angle.Pos_, leg[2].angle.Pos_, leg[3].angle.Pos_);

        // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m stdid %d %d \033[0m", Ethercat.packet_rx[0].can.StdId, Ethercat.packet_rx[1].can.StdId);
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    orthrus_control::OrthrusSystemHardware, hardware_interface::SystemInterface)
