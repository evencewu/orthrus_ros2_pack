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

        // 总里程计数据
        Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

        Eigen::Vector3d euler;
        Eigen::Vector3d angular_velocity;

        // 足端里程计估计
        Eigen::Vector3d foot_acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d foot_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d foot_velocity = Eigen::Vector3d::Zero();

        // imu估计
        Eigen::Vector3d imu_acceleration = Eigen::Vector3d::Zero();
        Eigen::Vector3d imu_position = Eigen::Vector3d::Zero();
        Eigen::Vector3d imu_velocity = Eigen::Vector3d::Zero();

        // 重力项
        Eigen::Vector3d gravity;

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
        Eigen::Vector3d target_position = Eigen::Vector3d(0, 0, 0.3);
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
        bool if_enable_calibration_encoder;
    };

    struct OrthrusInterfaces
    {
        struct RobotState robot_state;
        struct OdomState odom_state;
        struct RobotTarget robot_target;
        struct RobotCmd robot_cmd;
    };
}