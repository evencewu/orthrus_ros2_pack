#include "orthrus_controllers/OrthrusNmpc.hpp"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace orthrus_control
{
    TargetTrajectories targetPoseToTargetTrajectories(const vector_t &targetPose, const SystemObservation &observation,
                                                      const scalar_t &targetReachingTime)
    {
        // desired time trajectory
        const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

        // desired state trajectory
        vector_t currentPose = observation.state.segment<6>(6);
        currentPose(2) = COM_HEIGHT;
        currentPose(4) = 0;
        currentPose(5) = 0;
        vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
        stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
        stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

        // desired input trajectory (just right dimensions, they are not used)
        const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

        return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    TargetTrajectories goalToTargetTrajectories(const vector_t &goal, const SystemObservation &observation)
    {
        const vector_t currentPose = observation.state.segment<6>(6);
        const vector_t targetPose = [&]()
        {
            vector_t target(6);
            target(0) = goal(0);
            target(1) = goal(1);
            target(2) = COM_HEIGHT;
            target(3) = goal(3);
            target(4) = 0;
            target(5) = 0;
            return target;
        }();
        const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
        return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
    }

}