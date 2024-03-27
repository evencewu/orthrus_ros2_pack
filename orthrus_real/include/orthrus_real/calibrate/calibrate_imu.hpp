#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace orthrus_real::calibrate
{
    void GetEuler(Eigen::Quaterniond q, double *yaw, double *pitch, double *roll);
    Eigen::Quaterniond RotatingCoordinates(Eigen::Quaterniond input, double angle, Eigen::Vector3d axis,double install_angle, Eigen::Vector3d install_axis);
}