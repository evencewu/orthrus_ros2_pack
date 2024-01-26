#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace othrus_ctrl
{
    class OthrusCtrlNode : public rclcpp::Node
    {
    public:
        OthrusCtrlNode();
        //~OthrusCtrlNode() override;

        void main_loop();
    private:

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}