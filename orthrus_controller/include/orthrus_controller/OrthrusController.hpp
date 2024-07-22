#pragma once

#include <orthrus_controller/controller_base/OrthrusControllerBase.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace orthrus_controller
{
    class OrthrusController : public OrthrusControllerBase
    {

    public:
        ORTHRUS_CONTROLLER_PUBLIC
        OrthrusController();

    protected:
        
    };

}