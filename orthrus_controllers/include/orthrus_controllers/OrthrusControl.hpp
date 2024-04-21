#pragma once

#include <orthrus_interfaces/msg/orthrus_joint_control.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_state.hpp>
// #include <orthrus_controllers/visualization/OrthrusVisualization.hpp>

#include "orthrus_controllers/interface/OrthrusInterface.hpp"

// ros2
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <mutex>

// ocs2
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

namespace orthrus_control
{
    class OrthrusControlNode : public rclcpp::Node
    {
    public:
        OrthrusControlNode();
        // using CmdToTargetTrajectories = std::function<ocs2::TargetTrajectories(const ocs2::vector_t &cmd, const ocs2::SystemObservation &observation)>;

    private:
        std::shared_ptr<rclcpp::Node> node_ptr_;

        void MainLoop();

        // ocs2
        // rclcpp::Node::SharedPtr node_ = std::make_shared<OrthrusControlNode>();

        //  npc node
        // rclcpp::Node::SharedPtr node_;

        const std::string robotName = "orthrus";

        std::string taskFile_;
        std::string urdfFile_;
        std::string referenceFile_;

        std::shared_ptr<orthrus_control::OrthrusInterface> OrthrusInterfacePtr_;

        rclcpp::TimerBase::SharedPtr timer_;
    };
}