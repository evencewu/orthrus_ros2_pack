#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/PoseStamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>



namespace orthrus_control
{
    class OrthrusControlNode : public rclcpp::Node
    {
    public:
        using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t &cmd, const SystemObservation &observation)>;

        OrthrusControlNode();

    private:
        CmdToTargetTrajectories goalToTargetTrajectories;
        CmdToTargetTrajectories cmdVelToTargetTrajectories;

        ocs2::SystemObservation latestObservation_;
    };
}