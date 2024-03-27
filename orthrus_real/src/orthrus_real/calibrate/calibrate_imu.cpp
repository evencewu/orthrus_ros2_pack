#include "orthrus_real/calibrate/calibrate_imu.hpp"

namespace orthrus_real::calibrate
{
    void get_angle(Eigen::Quaterniond q, double *yaw, double *pitch, double *roll)
    {
        Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

        // 从旋转矩阵中提取欧拉角（这里使用Z-Y-X顺序）

        // 如果sin(pitch)不为零，则可以避免除以零的情况
        if (std::abs(rotationMatrix(2, 0)) != 1)
        {
            *roll = std::atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
            *pitch = std::asin(-rotationMatrix(2, 0));
            *yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
        }
        else
        {
            // 当sin(pitch)接近零时，情况变得不确定，但我们仍然可以通过其他方式获取合理的解
            // 在这种情况下，我们可以选择roll和yaw的任意值，因为pitch已经是90度或-90度
            // 这里我们简单地将roll和yaw设为0
            *roll = 0;
            *pitch = std::copysign(M_PI / 2, -rotationMatrix(2, 0));
            *yaw = 0;
        }
    }

    Eigen::Quaterniond RotatingCoordinates(Eigen::Quaterniond input, double angle, Eigen::Vector3d axis,double install_angle, Eigen::Vector3d install_axis)
    {
        Eigen::AngleAxisd rotation(angle, axis); // 绕y轴旋转90度
        Eigen::Quaterniond q(rotation);

        Eigen::AngleAxisd install_rotation(install_angle,install_axis);
        Eigen::Quaterniond install_q(install_rotation);

        return q * input * install_q;
    }
}