#include "orthrus_controllers/OrthrusControl.hpp"

namespace orthrus_control
{
    OrthrusControlNode::OrthrusControlNode() : Node("orthrus_control", rclcpp::NodeOptions()
                                                                           .allow_undeclared_parameters(true)
                                                                           .automatically_declare_parameters_from_overrides(true))
    {

         

        node_ptr_ = shared_from_this;
        taskFile_ = this->get_parameter("taskFile").as_string();
        urdfFile_ = this->get_parameter("urdfFile").as_string();
        referenceFile_ = this->get_parameter("referenceFile").as_string();

        OrthrusInterfacePtr_ = std::make_shared<orthrus_control::OrthrusInterface>(node_ptr_);
        OrthrusInterfacePtr_->Init();

        // robot_interface_ptr_ = std::make_shared<ocs2::legged_robot::LeggedRobotInterface>(
        //     taskFile_, urdfFile_, referenceFile_);

        // orthrus_joint_control_pub_ = this->create_publisher<orthrus_interfaces::msg::OrthrusJointControl>("/orthrus_interface/joint_control", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&OrthrusControlNode::MainLoop, this));
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyClass::afterConstructor, this));
    }

    void OrthrusControlNode::MainLoop()
    {
        // UpdateRobotParma();
        // orthrus_joint_control_msg_ = PositonCtrl_.StandUp();
        // orthrus_joint_control_pub_->publish(orthrus_joint_control_msg_);
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
#include "rclcpp/rclcpp.hpp"

class MyClass : public rclcpp::Node {
public:
    MyClass() : Node("my_node"), count_(0) {
        // 创建一个定时器，在1秒后触发，并绑定到成员函数
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyClass::afterConstructor, this));
    }

private:
    void afterConstructor() {
        // 在此处执行你想要的特定函数
        RCLCPP_INFO(this->get_logger(), "Function called after constructor");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyClass>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}