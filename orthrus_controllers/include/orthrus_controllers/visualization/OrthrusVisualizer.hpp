/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>



#include "rclcpp/rclcpp.hpp"

namespace orthrus_control
{

    class OrthrusVisualizer : public ocs2::DummyObserver
    {
    public:
        /** Visualization settings (publicly available) */
        std::string frameId_ = "odom";                   // Frame name all messages are published in
        ocs2::scalar_t footMarkerDiameter_ = 0.03;       // Size of the spheres at the feet
        ocs2::scalar_t footAlphaWhenLifted_ = 0.3;       // Alpha value when a foot is lifted.
        ocs2::scalar_t forceScale_ = 1000.0;             // Vector scale in N/m
        ocs2::scalar_t velScale_ = 5.0;                  // Vector scale in m/s
        ocs2::scalar_t copMarkerDiameter_ = 0.03;        // Size of the sphere at the center of pressure
        ocs2::scalar_t supportPolygonLineWidth_ = 0.005; // LineThickness for the support polygon
        ocs2::scalar_t trajectoryLineWidth_ = 0.01;      // LineThickness for trajectories
        std::vector<ocs2::Color> feetColorMap_ = {
            ocs2::Color::blue, ocs2::Color::orange, ocs2::Color::yellow,
            ocs2::Color::purple}; // Colors for markers per feet

        /**
         *
         * @param pinocchioInterface
         * @param n
         * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
         */
        OrthrusVisualizer(
            ocs2::PinocchioInterface pinocchioInterface,
            ocs2::CentroidalModelInfo centroidalModelInfo,
            const ocs2::PinocchioEndEffectorKinematics &endEffectorKinematics,
            const rclcpp::Node::SharedPtr &node, ocs2::scalar_t maxUpdateFrequency = 100.0);

        ~OrthrusVisualizer() override = default;

        void update(const ocs2::SystemObservation &observation,
                    const ocs2::PrimalSolution &primalSolution,
                    const ocs2::CommandData &command) override;

        void publishTrajectory(
            const std::vector<ocs2::SystemObservation> &system_observation_array,
            ocs2::scalar_t speed = 1.0);

        void publishObservation(rclcpp::Time timeStamp,
                                const ocs2::SystemObservation &observation);

        void publishDesiredTrajectory(rclcpp::Time timeStamp,
                                      const ocs2::TargetTrajectories &targetTrajectories);

        void publishOptimizedStateTrajectory(rclcpp::Time timeStamp,
                                             const ocs2::scalar_array_t &mpcTimeTrajectory,
                                             const ocs2::vector_array_t &mpcStateTrajectory,
                                             const ocs2::ModeSchedule &modeSchedule);

    protected:
        rclcpp::Node::SharedPtr node_;

    private:
        OrthrusVisualizer(const OrthrusVisualizer &) = delete;
        void publishJointTransforms(rclcpp::Time timeStamp,
                                    const ocs2::vector_t &jointAngles) const;
        void publishBaseTransform(rclcpp::Time timeStamp, const ocs2::vector_t &basePose);
        void publishCartesianMarkers(rclcpp::Time timeStamp,
                                     const switched_model::contact_flag_t &contactFlags,
                                     const std::vector<switched_model::vector3_t> &feetPositions,
                                     const std::vector<switched_model::vector3_t> &feetForces) const;

        ocs2::PinocchioInterface pinocchioInterface_;
        const ocs2::CentroidalModelInfo centroidalModelInfo_;
        std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

        tf2_ros::TransformBroadcaster tfBroadcaster_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
            costDesiredBasePositionPublisher_;
        std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>
            costDesiredFeetPositionPublishers_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            stateOptimizedPublisher_;

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            currentStatePublisher_;

        ocs2::scalar_t lastTime_;
        ocs2::scalar_t minPublishTimeDifference_;
    };

}
