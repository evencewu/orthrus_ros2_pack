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

        JointParma() : position(12, 0.0), velocity(12, 0.0), effort(12, 0.0)
        {
            // 可选的，可以在构造函数中添加额外的初始化逻辑
            // 例如：可以在这里进一步初始化成员变量或执行其他操作
        }
    };

    struct Odom
    {
    };

    struct Touch
    {
        bool sensor[4];
    };
}
