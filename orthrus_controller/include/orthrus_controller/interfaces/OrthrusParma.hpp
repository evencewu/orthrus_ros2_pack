#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace orthrus_controller
{
    struct JointParma
    {
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;

        JointParma() : position(12, 0.0), velocity(12, 0.0), effort(12, 0.0){}
    };

    struct Odom
    {

    };

    struct Imu
    {

    };

    struct Touch
    {
        bool sensor[4];
    };
}
