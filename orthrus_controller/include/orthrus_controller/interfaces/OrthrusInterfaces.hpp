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
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;

        ImuState imu;
        ImuState imu_last;

        // 欧拉角
        Eigen::Vector3d euler;

        TouchState touch_state[4];
    };

    struct RobotTarget
    {
        Eigen::Vector3d target_velocity;
        Eigen::Vector3d target_position;

        Eigen::Quaterniond target_attitude;

        Eigen::VectorXd target_body_force = Eigen::VectorXd::Zero(6);

        std::vector<std::vector<bool>> gait_sequence = {{1, 1, 1, 1},{1, 0, 0, 1}, {0, 1, 1, 0}};

        int gait_num;
        bool if_stand;

        RobotTarget()
        {
            gait_num = 0;
        }
    };

    struct RobotCmd
    {
        Eigen::VectorXd effort;
    };

    class OrthrusInterfaces
    {
    public:
        struct RobotState robot_state;
        struct OdomState odom_state;
        struct RobotTarget robot_target;
        struct RobotCmd robot_cmd;
    };
}