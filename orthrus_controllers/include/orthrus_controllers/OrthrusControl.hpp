#pragma once

#include "rclcpp/rclcpp.hpp"

#include <orthrus_interfaces/msg/orthrus_joint_control.hpp>
#include <orthrus_interfaces/msg/orthrus_joint_state.hpp>

// #include <orthrus_controllers/visualization/OrthrusVisualization.hpp>

#include "orthrus_controllers/interface/OrthrusInterface.hpp"
#include "orthrus_controllers/interface/OrthrusHwInterface.hpp"

// ros2
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <mutex>

// ocs2
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>

#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

#include "orthrus_controllers/visualization/OrthrusVisualization.hpp"



namespace orthrus_control
{
    class OrthrusControlNode : public rclcpp::Node
    {
    public:
        OrthrusControlNode();
        // using CmdToTargetTrajectories = std::function<ocs2::TargetTrajectories(const ocs2::vector_t &cmd, const ocs2::SystemObservation &observation)>;

    private:
        std::shared_ptr<rclcpp::Node> node_ptr_;

        bool init_flag_ = false;
        void Init();
        void Starting();
        void MainLoop();
        void updateStateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period);

        void MpcInit();
        void MrtInit();
        // void Init();

        // ocs2
        // rclcpp::Node::SharedPtr node_ = std::make_shared<OrthrusControlNode>();

        //  npc node
        // rclcpp::Node::SharedPtr node_;

        const std::string robotName = "orthrus";

        std::string taskFile_;
        std::string urdfFile_;
        std::string referenceFile_;

        // State Estimation
        ocs2::SystemObservation currentObservation_;
        ocs2::vector_t measuredRbdState_;
        std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;

        // Interface
        std::shared_ptr<OrthrusInterface> InterfacePtr_;
        std::shared_ptr<OrthrusHwInterface> OrthrusInterfacePtr_;
        std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        /*MPC*/
        std::shared_ptr<ocs2::MPC_BASE> mpc_;
        std::shared_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface_;

        //Visualization
        std::shared_ptr<ocs2::legged_robot::LeggedRobotVisualizer> robotVisualizer_;
        std::shared_ptr<OrthrusVisualization> selfCollisionVisualization_;

        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpcObservationPublisher_;

        std::thread mpcThread_;
        std::atomic_bool controllerRunning_{}, mpcRunning_{};

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr init_timer_;

        ocs2::benchmark::RepeatedTimer mpcTimer_;
    };
}