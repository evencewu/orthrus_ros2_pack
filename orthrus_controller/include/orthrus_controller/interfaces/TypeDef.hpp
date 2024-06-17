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
    struct ImuState
    {
        Eigen::Vector3d angular_velocity;
        Eigen::Vector3d linear_acceleration;
        Eigen::Quaterniond orientation;
    };

    struct JointState
    {
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;

        JointState() : position(12, 0.0), velocity(12, 0.0), effort(12, 0.0) {}
    };

    struct TouchState
    {
        bool sensor;
        Eigen::Vector3d touch_position;
        Eigen::Matrix3d touch_rotation;
        Eigen::Vector3d touch_force;

        TouchState() : sensor(false), touch_position(Eigen::Vector3d::Zero()), touch_force(Eigen::Vector3d::Zero()) {}
    };



    //struct GaitSequence
    //{
    //    std::vector<std::vector<bool>> vec = {1,0,0,1};
    //    std::vector<bool> vec = {1,0,0,1};
    //    std::vector<bool> vec = {1,0,0,1};
    //}


}