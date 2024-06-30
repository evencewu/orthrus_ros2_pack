#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "orthrus_control/ethercat/EcatBase.hpp"
#include "orthrus_control/ethercat/TypeDef.hpp"
#include "orthrus_control/assembly/Leg.hpp"
#include "orthrus_control/assembly/Imu.hpp"

/// @brief ps.这里的variable储存的都是经过转换的标准化数据，并非原始数据
namespace orthrus_control
{
    struct LegImuVariable
    {
        Eigen::Quaterniond attitude;
    };

    struct BodyImuVariable
    {
        Eigen::Quaterniond attitude;
        Eigen::Vector3d acceleration;
        Eigen::Vector3d angle_speed;
    };

    struct MotorVariable
    {
        double position;
        double velocity;
        double acceleration;
    };

    struct LegVariable
    {
        MotorVariable motor[3];
        LegImuVariable leg_imu;
    };

    struct OrthrusControlVariable
    {
        Leg leg[4];
        Imu body_imu;
        // LegVariable leg[4];
        // BodyImuVariable body_imu;
    };
}