#include "orthrus_controllers/OrthrusNmpc.hpp"

namespace orthrus_control
{
    namespace
    {
        ocs2::scalar_t TARGET_DISPLACEMENT_VELOCITY =0.5;
        ocs2::scalar_t TARGET_ROTATION_VELOCITY =1.57;
        ocs2::scalar_t COM_HEIGHT = 0.3;
        ocs2::vector_t DEFAULT_JOINT_STATE(12);
        ocs2::scalar_t TIME_TO_TARGET = 1.0;
    } // namespace

    ocs2::scalar_t estimateTimeToTarget(const ocs2::vector_t &desiredBaseDisplacement)
    {
        const ocs2::scalar_t &dx = desiredBaseDisplacement(0);
        const ocs2::scalar_t &dy = desiredBaseDisplacement(1);
        const ocs2::scalar_t &dyaw = desiredBaseDisplacement(3);
        const ocs2::scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
        const ocs2::scalar_t displacement = std::sqrt(dx * dx + dy * dy);
        const ocs2::scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
        return std::max(rotationTime, displacementTime);
    }

    ocs2::TargetTrajectories targetPoseToTargetTrajectories(const ocs2::vector_t &targetPose, const ocs2::SystemObservation &observation,
                                                            const ocs2::scalar_t &targetReachingTime)
    {
        // desired time trajectory
        const ocs2::scalar_array_t timeTrajectory{observation.time, targetReachingTime};

        // desired state trajectory
        ocs2::vector_t currentPose = observation.state.segment<6>(6);
        currentPose(2) = COM_HEIGHT;
        currentPose(4) = 0;
        currentPose(5) = 0;
        ocs2::vector_array_t stateTrajectory(2, ocs2::vector_t::Zero(observation.state.size()));
        stateTrajectory[0] << ocs2::vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
        stateTrajectory[1] << ocs2::vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

        // desired input trajectory (just right dimensions, they are not used)
        const ocs2::vector_array_t inputTrajectory(2, ocs2::vector_t::Zero(observation.input.size()));

        return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    ocs2::TargetTrajectories goalToTargetTrajectories(const ocs2::vector_t &goal, const ocs2::SystemObservation &observation)
    {
        const ocs2::vector_t currentPose = observation.state.segment<6>(6);
        const ocs2::vector_t targetPose = [&]()
        {
            ocs2::vector_t target(6);
            target(0) = goal(0);
            target(1) = goal(1);
            target(2) = COM_HEIGHT;
            target(3) = goal(3);
            target(4) = 0;
            target(5) = 0;
            return target;
        }();
        const ocs2::scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
        return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
    }

}