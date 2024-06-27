#pragma once

#include "orthrus_controller/interfaces/TypeDef.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace orthrus_controller
{
    struct RobotState
    {
        JointState joint;

        ImuState body_imu;
        ImuState leg_imu[4];
    };

    struct OdomState
    {
        double dt;

        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

        Eigen::Vector3d gravity;

        // 欧拉角
        Eigen::Vector3d euler;
        Eigen::Vector3d angular_velocity;

        ImuState imu;
        ImuState imu_last;

        TouchState touch_state[4];

        OdomState()
        {
            dt = 0.0;
            gravity << 0, 0, -9.8;
        }
    };

    struct RobotTarget
    {
        Eigen::Vector3d target_position;
        Eigen::Vector3d target_velocity;

        Eigen::Quaterniond target_attitude;
        Eigen::Vector3d target_euler;
        Eigen::Vector3d target_angular_velocity;

        Eigen::VectorXd target_body_force = Eigen::VectorXd::Zero(6);

        std::vector<std::vector<bool>> gait_sequence = {{1, 1, 1, 1}, {1, 0, 0, 1}, {0, 1, 1, 0}};

        int gait_num = 0;

        bool if_stand;
        bool if_enable;
    };

    struct RobotCmd
    {
        Eigen::VectorXd effort;

        bool if_enable_power;
        bool if_enable_calibration;
    };

    struct OrthrusInterfaces
    {
        struct RobotState robot_state;
        struct OdomState odom_state;
        struct RobotTarget robot_target;
        struct RobotCmd robot_cmd;
    };
}