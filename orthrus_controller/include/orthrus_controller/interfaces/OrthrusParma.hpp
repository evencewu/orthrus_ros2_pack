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
    };
    
    struct Odom
    {

    };

    struct Touch
    {
        bool sensor[4];
    };
}
