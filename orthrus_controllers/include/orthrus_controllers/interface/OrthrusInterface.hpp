

#pragma once

#include "rclcpp/rclcpp.hpp"

// ocs2
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_sqp/SqpSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_legged_robot/common/ModelSettings.h>

#include "ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace orthrus_control
{
    using namespace ocs2;
    using namespace legged_robot;

    class OrthrusInterface final : public RobotInterface
    {
    public:
        /**
         * Constructor
         *
         * @throw Invalid argument error if input task file or urdf file does not exist.
         *
         * @param [in] taskFile: The absolute path to the configuration file for the MPC.
         * @param [in] urdfFile: The absolute path to the URDF file for the robot.
         * @param [in] referenceFile: The absolute path to the reference configuration file.
         * @param [in] useHardFrictionConeConstraint: Which to use hard or soft friction cone constraints.
         */
        OrthrusInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                         bool useHardFrictionConeConstraint = false);

        ~OrthrusInterface() override = default;

        virtual void setupOptimalControlProblem(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                                bool verbose);

        const OptimalControlProblem &getOptimalControlProblem() const override { return *problemPtr_; }

        const ModelSettings &modelSettings() const { return modelSettings_; }
        const ddp::Settings &ddpSettings() const { return ddpSettings_; }
        const mpc::Settings &mpcSettings() const { return mpcSettings_; }
        const rollout::Settings &rolloutSettings() const { return rolloutSettings_; }
        const sqp::Settings &sqpSettings() { return sqpSettings_; }
        const ipm::Settings &ipmSettings() { return ipmSettings_; }

        const vector_t &getInitialState() const { return initialState_; }
        const RolloutBase &getRollout() const { return *rolloutPtr_; }
        PinocchioInterface &getPinocchioInterface() { return *pinocchioInterfacePtr_; }
        const CentroidalModelInfo &getCentroidalModelInfo() const { return centroidalModelInfo_; }
        PinocchioGeometryInterface &getGeometryInterface() { return *geometryInterfacePtr_; }
        std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

        const Initializer &getInitializer() const override { return *initializerPtr_; }
        std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

    protected:
        virtual void setupModel(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose);
        virtual void setupReferenceManager(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                           bool verbose);
        virtual void setupPreComputation(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                         bool verbose);

        std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string &file, bool verbose) const;

        std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string &taskFile, const CentroidalModelInfo &info, bool verbose);
        matrix_t initializeInputCostWeight(const std::string &taskFile, const CentroidalModelInfo &info);

        std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string &taskFile, bool verbose) const;
        std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);
        std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                      const RelaxedBarrierPenalty::Config &barrierPenaltyConfig);

        std::unique_ptr<EndEffectorKinematics<scalar_t>> getEeKinematicsPtr(const std::vector<std::string> &footNames,
                                                                            const std::string &modelName);

        std::unique_ptr<StateInputConstraint> getZeroForceConstraint(size_t contactPointIndex);
        std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                        size_t contactPointIndex);
        std::unique_ptr<StateCost> getSelfCollisionConstraint(const PinocchioInterface &pinocchioInterface, const std::string &taskFile,
                                                              const std::string &prefix, bool verbose);

        ModelSettings modelSettings_;
        ddp::Settings ddpSettings_;
        mpc::Settings mpcSettings_;
        sqp::Settings sqpSettings_;
        ipm::Settings ipmSettings_;
        const bool useHardFrictionConeConstraint_;

        std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
        CentroidalModelInfo centroidalModelInfo_;
        std::unique_ptr<PinocchioGeometryInterface> geometryInterfacePtr_;

        std::unique_ptr<OptimalControlProblem> problemPtr_;
        std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

        rollout::Settings rolloutSettings_;
        std::unique_ptr<RolloutBase> rolloutPtr_;
        std::unique_ptr<Initializer> initializerPtr_;

        vector_t initialState_;
    };

}
