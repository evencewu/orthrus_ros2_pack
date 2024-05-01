#include "orthrus_controllers/OrthrusControl.hpp"

namespace orthrus_control
{
    OrthrusControlNode::OrthrusControlNode() : Node("orthrus_control", rclcpp::NodeOptions()
                                                                           .allow_undeclared_parameters(true)
                                                                           .automatically_declare_parameters_from_overrides(true))
    {
        // robot_interface_ptr_ = std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(
        //     taskFile_, urdfFile_, referenceFile_);

        // orthrus_joint_control_pub_ = this->create_publisher<orthrus_interfaces::msg::OrthrusJointControl>("/orthrus_interface/joint_control", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&OrthrusControlNode::MainLoop, this));
        init_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&OrthrusControlNode::Init, this));
    }

    void OrthrusControlNode::MainLoop()
    {
        if (init_flag_)
        {
            // RCLCPP_INFO(node_ptr_->get_logger(), "loop");
            //  UpdateRobotParma();
            //  orthrus_joint_control_msg_ = PositonCtrl_.StandUp();
            //  orthrus_joint_control_pub_->publish(orthrus_joint_control_msg_);
        }
    }

    void OrthrusControlNode::Init()
    {
        if (!init_flag_)
        {
            node_ptr_ = shared_from_this();

            taskFile_ = this->get_parameter("taskFile").as_string();
            urdfFile_ = this->get_parameter("urdfFile").as_string();
            referenceFile_ = this->get_parameter("referenceFile").as_string();

            bool verbose = false;
            loadData::loadCppDataType(taskFile_, "legged_robot_interface.verbose", verbose);

            HybridJointInterfacesPtr_ = std::make_shared<HybridJointInterfaces>(node_ptr_);

            // Robot interface
            InterfacePtr_ = std::make_shared<OrthrusInterface>(taskFile_, urdfFile_, referenceFile_);
            InterfacePtr_->setupOptimalControlProblem(taskFile_, urdfFile_, referenceFile_, verbose);

            MpcInit();
            MrtInit();

            // Visualization
            ocs2::CentroidalModelPinocchioMapping pinocchioMapping(InterfacePtr_->getCentroidalModelInfo());
            eeKinematicsPtr_ = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(InterfacePtr_->getPinocchioInterface(), pinocchioMapping,
                                                                                      InterfacePtr_->modelSettings().contactNames3DoF);
            robotVisualizer_ = std::make_shared<ocs2::legged_robot::LeggedRobotVisualizer>(InterfacePtr_->getPinocchioInterface(),
                                                                                           InterfacePtr_->getCentroidalModelInfo(), *eeKinematicsPtr_, node_ptr_);

            selfCollisionVisualization_.reset(new OrthrusVisualization(InterfacePtr_->getPinocchioInterface(), InterfacePtr_->getGeometryInterface(), pinocchioMapping, node_ptr_));

            // State Estimation
            stateEstimate_ = std::make_shared<StateEstimateBase>(InterfacePtr_->getPinocchioInterface(),
                                                                 InterfacePtr_->getCentroidalModelInfo(), *eeKinematicsPtr_, node_ptr_);
            currentObservation_.time = 0;

            // Starting();

            init_flag_ = true;
        }
    }

    void OrthrusControlNode::Starting()
    {
        currentObservation_.state.setZero(InterfacePtr_->getCentroidalModelInfo().stateDim);
        updateStateEstimation(this->get_clock()->now(), rclcpp::Duration(0, 2000000)); // 0.005 sec
        currentObservation_.input.setZero(InterfacePtr_->getCentroidalModelInfo().inputDim);
        currentObservation_.mode = ModeNumber::STANCE;

        ocs2::TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

        mpcMrtInterface_->setCurrentObservation(currentObservation_);
        mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);

        RCLCPP_INFO(this->get_logger(), "Waiting for the initial policy ...");
        rclcpp::Rate leggedRate(InterfacePtr_->mpcSettings().mrtDesiredFrequency_);
        while (!mpcMrtInterface_->initialPolicyReceived() && rclcpp::ok())
        {
            mpcMrtInterface_->advanceMpc();
            leggedRate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Initial policy has been received.");

        mpcRunning_ = true;
    }

    void OrthrusControlNode::MpcInit()
    {
        mpc_ = std::make_shared<ocs2::SqpMpc>(InterfacePtr_->mpcSettings(), InterfacePtr_->sqpSettings(),
                                              InterfacePtr_->getOptimalControlProblem(), InterfacePtr_->getInitializer());
        rbdConversions_ = std::make_shared<ocs2::CentroidalModelRbdConversions>(InterfacePtr_->getPinocchioInterface(),
                                                                                InterfacePtr_->getCentroidalModelInfo());
        // Gait receiver
        auto gaitReceiverPtr = std::make_shared<ocs2::legged_robot::GaitReceiver>(
            node_ptr_, InterfacePtr_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
            robotName);

        auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(
            robotName, InterfacePtr_->getReferenceManagerPtr());
        rosReferenceManagerPtr->subscribe(node_ptr_);

        mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
        mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

        mpcObservationPublisher_ = node_ptr_->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);

        //
        // observationPublisher_ = nh.advertise<ocs2_msgs::msg::mpc_observation>(robotName + "_mpc_observation", 1);
    }

    void OrthrusControlNode::MrtInit()
    {
        mpcMrtInterface_ = std::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
        mpcMrtInterface_->initRollout(&InterfacePtr_->getRollout());
        mpcTimer_.reset();

        mpcThread_ = std::thread([&]()
                                 {
            while (controllerRunning_) 
            {
                try 
                {
                    ocs2::executeAndSleep(
                    [&]() {
                      if (mpcRunning_) {
                        mpcTimer_.startTimer();
                        mpcMrtInterface_->advanceMpc();
                        mpcTimer_.endTimer();
                      }
                    },
                    InterfacePtr_->mpcSettings().mpcDesiredFrequency_);
                } 
                catch (const std::exception& e) 
                {
                    controllerRunning_ = false;
                    //ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
                    //stopRequest(ros::Time());
                }
            } });

        ocs2::setThreadPriority(InterfacePtr_->sqpSettings().threadPriority, mpcThread_);
    }

    void OrthrusControlNode::updateStateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // measuredRbdState_ = stateEstimate_->update(time, period);
        ocs2::vector_t jointPos(HybridJointInterfacesPtr_->joint_num_), jointVel(HybridJointInterfacesPtr_->joint_num_);
        for (size_t i = 0; i <HybridJointInterfacesPtr_->joint_num_; ++i)
        {
            jointPos(i) = HybridJointInterfacesPtr_->getPosition(i);
            jointVel(i) = HybridJointInterfacesPtr_->getVelocity(i);
        }

        stateEstimate_->UpdateJointStates(jointPos, jointVel);
        //stateEstimate_->updateAngular(const vector3_t &zyx, const vector_t &angularVel);
        //stateEstimate_->updateLinear(const vector_t &pos, const vector_t &linearVel);

        currentObservation_.time += period.seconds();
        currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
        currentObservation_.mode = 0;
        // currentObservation_.mode = stateEstimate_->getMode();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_control::OrthrusControlNode>());
    rclcpp::shutdown();
    return 0;
}
