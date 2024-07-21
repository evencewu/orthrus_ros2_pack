#include "rsf_robot_interface/RobotInterface.hpp"
#include "rsf_pinocchio_interface/PinocchioInterface.hpp"

namespace orthrus_controller {
    using namespace rsf;

    class OrthrusInterface : public RobotInterface{
    
    protected:

        std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    };
}