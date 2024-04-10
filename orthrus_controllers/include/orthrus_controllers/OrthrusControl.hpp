#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <mutex>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace orthrus_control
{
    class OrthrusControlNode : public rclcpp::Node
    {
    public:
        OrthrusControlNode();
    private:
    
        ocs2::SystemObservation latestObservation_;
    };
}