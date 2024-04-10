#include "orthrus_controllers/OrthrusControl.hpp"

namespace orthrus_control
{
    OrthrusControlNode::OrthrusControlNode()
        : Node("orthrus_control")
    {
        ocs2::vector_t cmdGoal = ocs2::vector_t::Zero(6);
        //cmdGoal[0] = pose.pose.position.x;
        //cmdGoal[1] = pose.pose.position.y;
        //cmdGoal[2] = pose.pose.position.z;
        //Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
        //cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
        //cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
        //cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();
        cmdGoal[0] = 0;
        cmdGoal[1] = 0;
        cmdGoal[2] = 0.3;
        cmdGoal[3] = 0;
        cmdGoal[4] = 0;
        cmdGoal[5] = 0;

        //const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
        //targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_control::OrthrusControlNode>());
    rclcpp::shutdown();
    return 0;
}