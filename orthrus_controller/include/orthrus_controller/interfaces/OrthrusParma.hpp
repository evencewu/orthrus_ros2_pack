#pragma once

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
    struct JointState
    {
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;

        JointState() : position(12, 0.0), velocity(12, 0.0), effort(12, 0.0) {}
    };

    struct ImuState
    {
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
        Eigen::Quaterniond orientation;
    };

    struct OdomState
    {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;

        ImuState imu;
        ImuState imu_last;

        // 欧拉角
        Eigen::Vector3d euler;
    };

    struct TouchState
    {
        bool sensor;
        Eigen::Vector3d touch_position;
        Eigen::Vector3d touch_force;

        TouchState() : sensor(false), touch_position(Eigen::Vector3d::Zero()),touch_force(Eigen::Vector3d::Zero())
        {

        }
    };

    struct MpcTarget
    {
        Eigen::Vector3d target_velocity;

        Eigen::Vector3d target_position;
        Eigen::Quaterniond target_attitude;

        enum class Gait
        {
            STAND,
            LF_RH,
            LH_RF,
        } gait;

        MpcTarget()
        {
            target_velocity = Eigen::Vector3d::Zero();
            target_position = Eigen::Vector3d::Zero();
            target_attitude = Eigen::Quaterniond::Identity();

            gait = Gait::STAND;
        }
    };
}
