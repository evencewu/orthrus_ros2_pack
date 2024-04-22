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
    }

    void OrthrusControlNode::MainLoop()
    {
        if (!init_flag_)
        {
            Init();
            init_flag_ = true;
        }

        // RCLCPP_INFO(node_ptr_->get_logger(), "loop");
        //  UpdateRobotParma();
        //  orthrus_joint_control_msg_ = PositonCtrl_.StandUp();
        //  orthrus_joint_control_pub_->publish(orthrus_joint_control_msg_);
    }

    void OrthrusControlNode::Init()
    {
        node_ptr_ = shared_from_this();

        taskFile_ = this->get_parameter("taskFile").as_string();
        urdfFile_ = this->get_parameter("urdfFile").as_string();
        referenceFile_ = this->get_parameter("referenceFile").as_string();

        OrthrusInterfacePtr_ = std::make_shared<orthrus_control::OrthrusInterface>(node_ptr_);
        OrthrusInterfacePtr_->Init();

        // Robot interface
        InterfacePtr_ = std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(taskFile_, urdfFile_, referenceFile_);

        MpcInit();

        // OrthrusVisualization(interface.getPinocchioInterface(), interface.getGeometryInterface(), pinocchioMapping);

        // mpc_
        //
        // ocs2::SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(),
        //                  interface.getOptimalControlProblem(), interface.getInitializer());
        // mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
        // mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

        // ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
        // mpcNode.launchNodes(node_ptr_);
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

        mpcObservationPublisher_ = node_ptr_ ->create_publisher<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1);

        //
        // observationPublisher_ = nh.advertise<ocs2_msgs::msg::mpc_observation>(robotName + "_mpc_observation", 1);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_control::OrthrusControlNode>());
    rclcpp::shutdown();
    return 0;
}
/*
int main(int argc, char *argv[])
{
    const std::string robotName = "orthrus";

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        robotName + "_node",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    const std::string taskFile = node->get_parameter("taskFile").as_string();
    const std::string urdfFile = node->get_parameter("urdfFile").as_string();
    const std::string referenceFile = node->get_parameter("referenceFile").as_string();

    // Robot Real Interface
    auto OrthrusInterfacePtr = std::make_shared<orthrus_control::OrthrusInterface>();
    OrthrusInterfacePtr->Init(node);

    // Robot interface
    ocs2::legged_robot::LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);

    // Gait receiver
    auto gaitReceiverPtr = std::make_shared<ocs2::legged_robot::GaitReceiver>(
        node, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(),
        robotName);

    auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(
        robotName, interface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(node);

    //OrthrusVisualization(interface.getPinocchioInterface(), interface.getGeometryInterface(), pinocchioMapping);

    ocs2::SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(),
                     interface.getOptimalControlProblem(), interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

    ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
    mpcNode.launchNodes(node);

    return 0;
}
*/
