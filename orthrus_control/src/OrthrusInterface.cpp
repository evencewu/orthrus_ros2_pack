#include "orthrus_control/OrthrusInterface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
namespace orthrus_control
{
    hardware_interface::CallbackReturn OrthrusSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "OrthrusHardware on init");

        this->node_ = std::make_shared<rclcpp::Node>("OrthrusHardware");

        // hardware init
        assembly_ = std::make_shared<OrthrusControlVariable>();

        assembly_->leg[0].Init(IMU1, USART6, 1);
        assembly_->leg[1].Init(IMU2, USART1, 1);
        assembly_->leg[2].Init(IMU4, USART6, 0);
        assembly_->leg[3].Init(IMU3, USART1, 0);
        assembly_->body_imu.Init(IMU5, 1);

        // calibration_visualization = std::make_shared<CalibrationVisualization>(this->node_);
        // calibration_visualization->Init(assembly_);

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
        hw_sensor_states_.resize(10, std::numeric_limits<double>::quiet_NaN());
        hw_leg_imu_sensor_states_.resize(16, std::numeric_limits<double>::quiet_NaN());

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

        for (uint j = 0; j < info_.sensors[0].state_interfaces.size(); j++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.sensors[0].name, info_.sensors[0].state_interfaces[j].name, &hw_sensor_states_[j]));
        }

        for (uint i = 1; i < 5; i++)
        {
            for (uint j = 0; j < info_.sensors[i].state_interfaces.size(); j++)
            {
                // RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m i = %d j = %d \033[0m", i, j);
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_leg_imu_sensor_states_[(i - 1) * 4 + j]));
            }
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

        RCLCPP_INFO(rclcpp::get_logger("OrthrusHardware"), "\033[33m Activating ...please wait...\033[0m");

        // set some default values
        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_positions_[i] = 0;
                hw_velocities_[i] = 0;
                hw_effort_[i] = 0;
                hw_commands_[i] = 0;
            }
        }

        for (auto i = 0u; i < hw_sensor_states_.size(); i++)
        {
            hw_sensor_states_[i] = 0;
        }

        for (auto i = 0u; i < hw_leg_imu_sensor_states_.size(); i++)
        {
            hw_leg_imu_sensor_states_[i] = 0;
        }

        gpio_commands_["enable_power"] = 0;
        gpio_commands_["calibration_position"] = 0;
        gpio_commands_["calibration_encoder"] = 0;

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
        const rclcpp::Time &time, const rclcpp::Duration &period)
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

        ethercat_prepare_flag_ = Ethercat.EcatSyncMsg();

        auto motordata = PrepairMotorData();

        PrepairSensorData();

        hw_positions_ = motordata[0];
        hw_velocities_ = motordata[1];
        hw_effort_ = motordata[2];

        hw_sensor_states_[4] = assembly_->body_imu.standard_angle_speed_[0];
        hw_sensor_states_[5] = assembly_->body_imu.standard_angle_speed_[1];
        hw_sensor_states_[6] = assembly_->body_imu.standard_angle_speed_[2];

        hw_sensor_states_[7] = assembly_->body_imu.standard_acc_[0];
        hw_sensor_states_[8] = assembly_->body_imu.standard_acc_[1];
        hw_sensor_states_[9] = assembly_->body_imu.standard_acc_[2];

        hw_sensor_states_[3] = assembly_->body_imu.unified_gyro_.w();
        hw_sensor_states_[0] = assembly_->body_imu.unified_gyro_.x();
        hw_sensor_states_[1] = assembly_->body_imu.unified_gyro_.y();
        hw_sensor_states_[2] = assembly_->body_imu.unified_gyro_.z();
        
        // for (int i = 0; i < 10; i++)
        //{
        //     hw_sensor_states_[i] = i;
        // }

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                hw_leg_imu_sensor_states_[i * 4] = assembly_->leg[i].imu.standard_gyro_.x();
                hw_leg_imu_sensor_states_[i * 4 + 1] = assembly_->leg[i].imu.standard_gyro_.y();
                hw_leg_imu_sensor_states_[i * 4 + 2] = assembly_->leg[i].imu.standard_gyro_.z();
                hw_leg_imu_sensor_states_[i * 4 + 3] = assembly_->leg[i].imu.standard_gyro_.w();
            }
        }

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

        Log();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type orthrus_control::OrthrusSystemHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        for (int motor_num = 0; motor_num < 12; motor_num++)
        {
            if (motor_num % 3 == 0)
            {
                command_effort[motor_num] = -hw_commands_[motor_num];
            }
            else if(motor_num % 3 == 1)
            {
               command_effort[motor_num] = -hw_commands_[motor_num]; 
            }
            else
            {
                command_effort[motor_num] = hw_commands_[motor_num];
            }
        }

        // calibration_visualization->Update(time);

        Update();

        return hardware_interface::return_type::OK;
    }

    void OrthrusSystemHardware::Update()
    {
        if (motorcan_send_flag_ < 12)
        {
            assembly_->leg[motorcan_send_flag_ / 3].motor[motorcan_send_flag_ % 3].SetOutput(&Ethercat.packet_tx[assembly_->leg[motorcan_send_flag_ / 3].slave_num_], 0, 0, 0, command_effort[motorcan_send_flag_], 0, 10);
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

        if (gpio_commands_["calibration_encoder"])
        {
            StartCalibrationEncoderPosition();
        }
        else
        {
            StopCalibrationEncoderPosition();
        }

        for (int leg_num = 0; leg_num < 4; leg_num++)
        {
            assembly_->leg[leg_num].Analyze(&Ethercat.packet_rx[assembly_->leg[leg_num].slave_num_]);
        }

        assembly_->body_imu.Analyze(&Ethercat.packet_rx[assembly_->body_imu.slave_num_]);
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    orthrus_control::OrthrusSystemHardware, hardware_interface::SystemInterface)
