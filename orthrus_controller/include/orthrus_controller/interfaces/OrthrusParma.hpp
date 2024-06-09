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
        ImuState imu;
        ImuState imu_last;
    };

    struct Touch
    {
        bool sensor[4];
    };
}
