#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

namespace orthrus_control
{
    class OrthrusNmpc
    {
        public:

        private:
            ocs2::legged_robot::LeggedRobotInterface interface;
    };
}