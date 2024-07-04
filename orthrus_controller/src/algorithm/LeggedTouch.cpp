#include "orthrus_controller/algorithm/LeggedTouch.hpp"

namespace orthrus_controller
{
    void LeggedTouch::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr,
                           std::shared_ptr<PinocchioInterfaces> pinocchio_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
        pinocchio_interfaces_ = pinocchio_ptr;
    }
}