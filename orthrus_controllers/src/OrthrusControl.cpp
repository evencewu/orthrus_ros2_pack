#include "orthrus_controllers/OrthrusControl.hpp"

namespace orthrus_control
{
    OrthrusControlNode::OrthrusControlNode() : Node("orthrus_control")
    {
        orthrus_joint_state_sub_ = this->create_subscription<orthrus_interfaces::msg::OrthrusJointState>("/orthrus_interface/joint_state", 10, std::bind(&OrthrusControlNode::OrthrusJointStateSubCallback, this, std::placeholders::_1));
        orthrus_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/orthrus_interface/imu", 10, std::bind(&OrthrusControlNode::OrthrusImuSubCallback, this, std::placeholders::_1));
        orthrus_joint_control_pub_ = this->create_publisher<orthrus_interfaces::msg::OrthrusJointControl>("/orthrus_interface/joint_control", 10);
        orthrus_viewer_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/orthrus_viewer/joint_state", 10);
        orthrus_viewer_horizontal_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&OrthrusControlNode::MainLoop, this));

        ocs2::vector_t cmdGoal = ocs2::vector_t::Zero(6);

        // cmdGoal[0] = pose.pose.position.x;
        // cmdGoal[1] = pose.pose.position.y;
        // cmdGoal[2] = pose.pose.position.z;
        // Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        // cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
        // cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
        // cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

        cmdGoal[0] = 0;
        cmdGoal[1] = 0;
        cmdGoal[2] = 0.3;
        cmdGoal[3] = 0;
        cmdGoal[4] = 0;
        cmdGoal[5] = 0;

        // const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
        // targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    }

    void OrthrusControlNode::MainLoop()
    {
        //UpdateRobotParma();
        //orthrus_joint_control_msg_ = PositonCtrl_.StandUp();
        //orthrus_joint_control_pub_->publish(orthrus_joint_control_msg_);
    }

    void OrthrusControlNode::OrthrusJointStateSubCallback(const orthrus_interfaces::msg::OrthrusJointState::SharedPtr msg)
    {
        orthrus_viewer_joint_state_msg_.header.stamp = this->now();
        orthrus_viewer_joint_state_msg_.header.frame_id = "body";

        std::vector<double> position = std::vector<double>(12);
        std::vector<double> velocity = std::vector<double>(12);
        std::vector<double> effort = std::vector<double>(12);

        orthrus_viewer_joint_state_msg_.name = joint_name_;

        for (int i = 0; i < 12; i++)
        {
            OrthrusParam_.joint[i].position = msg->motor[i].pos;
            OrthrusParam_.joint[i].velocity = msg->motor[i].vec;
            OrthrusParam_.joint[i].effort = msg->motor[i].torq;

            position[i] = msg->motor[i].pos;
            velocity[i] = msg->motor[i].vec;
            effort[i] = msg->motor[i].torq;
        }

        orthrus_viewer_joint_state_msg_.position = position;
        orthrus_viewer_joint_state_msg_.velocity = velocity;
        orthrus_viewer_joint_state_msg_.effort = effort;

        orthrus_viewer_joint_state_pub_->publish(orthrus_viewer_joint_state_msg_);
    }

    void OrthrusControlNode::OrthrusImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_stamped;
        // orthrus_viewer_horizontal_pub_
        tf_stamped.header.stamp = this->now();  

        tf_stamped.header.frame_id = "horizontal";
        tf_stamped.child_frame_id = "body";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;

        tf_stamped.transform.rotation.x = msg->orientation.x;
        tf_stamped.transform.rotation.y = msg->orientation.y;
        tf_stamped.transform.rotation.z = msg->orientation.z;
        tf_stamped.transform.rotation.w = msg->orientation.w;

        orthrus_viewer_horizontal_msg_.transforms.clear();

        orthrus_viewer_horizontal_msg_.transforms.push_back(tf_stamped);

        orthrus_viewer_horizontal_pub_->publish(orthrus_viewer_horizontal_msg_);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_control::OrthrusControlNode>());
    rclcpp::shutdown();
    return 0;
}