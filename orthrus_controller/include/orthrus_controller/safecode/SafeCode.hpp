#pragma once

#include "orthrus_controller/interfaces/OrthrusInterfaces.hpp"

namespace orthrus_controller
{
    class SafeCode
    {
    public:
        void Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr);

        bool PositionCheck();

    private:
        std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_;
    };

}