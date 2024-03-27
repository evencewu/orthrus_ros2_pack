#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace orthrus_real::calibrate
{
    typedef struct ImuData
    {
        double yaw;
        double pitch;
        double roll;
    } imu_data[3];

    void GetEuler(Eigen::Quaterniond q, double *yaw, double *pitch, double *roll);//M_PI

    void RotatingCoordinates();
}