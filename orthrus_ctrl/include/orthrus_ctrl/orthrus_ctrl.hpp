#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define PI 3.1415926

#define REAL 0
#define GAZEBO 1
#define ISSACSIM 2

#define ISSAC_K_P 10
#define ISSAC_K_D 1000

namespace othrus_ctrl
{
    struct IssacSimJoint
    {
        double effort;
        double position;
        double velocity;
    };

    class OthrusCtrlNode : public rclcpp::Node
    {
    public:
        OthrusCtrlNode();
        //~OthrusCtrlNode() override;

        void main_loop();

        // ISSACSIM
        void IssacSimTorqe();

        // REAL

    private:
        // ros2
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::JointState joint_state_;

        int interfaces_mode_;
        static const int kJointNum = 12;
        const std::vector<std::string> joint_name_ = {"hip_LB_joint", "leg1_LB_joint", "leg2_LB_joint",
                                                      "hip_LF_joint", "leg1_LF_joint", "leg2_LF_joint",
                                                      "hip_RB_joint", "leg1_RB_joint", "leg2_RB_joint",
                                                      "hip_RF_joint", "leg1_RF_joint", "leg2_RF_joint"};
        // ISSACSIM
        IssacSimJoint IssacSimJoint_[kJointNum];
    };
}