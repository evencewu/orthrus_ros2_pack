#include "orthrus_controller/interfaces/JoyInterface.hpp"

namespace orthrus_controller
{
    void JoyInterface::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
    }

    void JoyInterface::JoyCallback(const xbox_interfaces::msg::XboxControl::SharedPtr msg)
    {
        // mtx_.lock();
        // auto *cmd = &orthrus_interfaces_->robot_cmd;
        // 菜单键组合键用于不常用组合键
        if (msg->start == 1)
        {
            if (msg->x == 1)
            {
                orthrus_interfaces_->robot_cmd.if_enable_power = 1;
            }

            if (msg->y == 1)
            {
                orthrus_interfaces_->robot_cmd.if_enable_power = 0;
            }

            if (msg->b == 1)
            {
                orthrus_interfaces_->robot_cmd.if_enable_calibration = 1;
            }
            else
            {
                orthrus_interfaces_->robot_cmd.if_enable_calibration = 0;
            }

            if (msg->a == 1)
            {
                orthrus_interfaces_->robot_cmd.if_enable_calibration_encoder = 1;
            }
            else
            {
                orthrus_interfaces_->robot_cmd.if_enable_calibration_encoder = 0;
            }

            if (msg->lb == 1)
            {
                orthrus_interfaces_->robot_target.if_enable = 1;
            }

            if (msg->rb == 1)
            {
                orthrus_interfaces_->robot_target.if_enable = 0;
            }
        }
        else
        {
            if (msg->yy < 0)
            {
                if (orthrus_interfaces_->robot_target.target_position[2] < 0.30)
                {
                    orthrus_interfaces_->robot_target.target_position[2] += 0.01;
                }
            }

            if (msg->yy > 0)
            {
                if (orthrus_interfaces_->robot_target.target_position[2] > 0.03)
                {
                    orthrus_interfaces_->robot_target.target_position[2] -= 0.01;
                }
            }

            if (msg->xx < 0)
            {
                if (orthrus_interfaces_->robot_target.target_euler[0] < M_PI / 12)
                {
                    orthrus_interfaces_->robot_target.target_euler[0] += M_PI/120;
                }
            }

            if (msg->xx > 0)
            {
                if (orthrus_interfaces_->robot_target.target_euler[0] > M_PI / 12)
                {
                    orthrus_interfaces_->robot_target.target_euler[0] -= M_PI /120;
                }
            }

            orthrus_interfaces_->robot_target.target_euler[2] = RockerMapping(msg->rx, 32000, M_PI / 12, 2000);
            orthrus_interfaces_->robot_target.target_euler[1] = RockerMapping(-msg->ry, 32000, M_PI / 12, 2000);
        }
    }

    std::stringstream JoyInterface::GetJoyTarget()
    {
        std::stringstream ss;
        ss << "JoyInterface: " << orthrus_interfaces_->robot_target.gait_num << " " << std::endl;
        return ss;
    }

    double JoyInterface::RockerMapping(int input, int input_max, double output_max, int zero_zone)
    {
        double p_t = output_max / (input_max - zero_zone);
        double output;
        if (input > zero_zone)
        {
            output = p_t * (input - zero_zone);

            if (output > output_max)
            {
                output = output_max;
            }
        }
        else if (input < -zero_zone)
        {
            output = p_t * (input + zero_zone);

            if (output < -output_max)
            {
                output = -output_max;
            }
        }
        else
        {
            output = 0;
        }

        return output;
    }
}