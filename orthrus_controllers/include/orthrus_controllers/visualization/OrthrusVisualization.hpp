#pragma once

#include "rclcpp/rclcpp.hpp"
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_mpc/SystemObservation.h>
#include <utility>

namespace orthrus_control
{
    class OrthrusVisualization : public ocs2::GeometryInterfaceVisualization
    {
    public:
        OrthrusVisualization(ocs2::PinocchioInterface pinocchioInterface, ocs2::PinocchioGeometryInterface geometryInterface,
                             const ocs2::CentroidalModelPinocchioMapping &mapping,rclcpp::Node::SharedPtr& node, ocs2::scalar_t maxUpdateFrequency = 50.0)
            : mappingPtr_(mapping.clone()),
              GeometryInterfaceVisualization(std::move(pinocchioInterface), std::move(geometryInterface), node, "odom"),
              lastTime_(std::numeric_limits<ocs2::scalar_t>::lowest()),
              minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

        void update(const ocs2::SystemObservation &observation)
        {
            if (observation.time - lastTime_ > minPublishTimeDifference_)
            {
                lastTime_ = observation.time;

                publishDistances(mappingPtr_->getPinocchioJointPosition(observation.state));
            }
        }

    private:
        std::unique_ptr<ocs2::CentroidalModelPinocchioMapping> mappingPtr_;

        ocs2::scalar_t lastTime_;
        ocs2::scalar_t minPublishTimeDifference_;
    };

} // namespace legged
