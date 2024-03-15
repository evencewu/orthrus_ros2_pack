#include "orthrus_gazebo/orthrus_gazebo_interface.hpp"

namespace orthrus_gazebo
{
    orthrusGazeboNode::orthrusGazeboNode() : Node("orthrus_gazebo")
    {
        Init();

        joint_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/orthrus_gazebo_joint_effort_controller/commands", 10);

        orthrus_joint_state_pub_ = this->create_publisher<orthrus_interfaces::msg::OrthrusJointState>("/orthrus_interface/joint_state", 10);

        orthrus_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/orthrus_interface/imu", 10);

        orthrus_joint_control_sub_ = this->create_subscription<orthrus_interfaces::msg::OrthrusJointControl>("/orthrus_interface/joint_control", 10,
                                                                                                             std::bind(&orthrusGazeboNode::OrthrusJointControlCallback, this, std::placeholders::_1));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
                                                                                   std::bind(&orthrusGazeboNode::OrthrusGazeboJointStateCallback, this, std::placeholders::_1));

        orthrus_gazebo_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/orthrus_gazebo_imu/imu", 10,
                                                                                   std::bind(&orthrusGazeboNode::OrthrusGazeboImuCallback, this, std::placeholders::_1));
    }

    void orthrusGazeboNode::Init()
    {
        for (int i = 0; i < 12; i++)
        {
            JointParam_[i].name = joint_name_[i];
        }
    }

    void orthrusGazeboNode::OrthrusJointControlCallback(const orthrus_interfaces::msg::OrthrusJointControl::SharedPtr msg)
    {
        for (int i = 0; i < 12; i++)
        {
            hybrid_output_[i] = msg->motor_cmd[i].k_p * (msg->motor_cmd[i].target_p - orthrus_joint_state_msg_.motor[i].pos) + msg->motor_cmd[i].k_d * (msg->motor_cmd[i].target_d - orthrus_joint_state_msg_.motor[i].vec) + msg->motor_cmd[i].torqe;
        }

        joint_torque_msg_.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        joint_torque_msg_.layout.dim[0].size = 12;

        joint_torque_msg_.data = hybrid_output_;

        joint_torque_pub_->publish(joint_torque_msg_);
    }

    void orthrusGazeboNode::OrthrusGazeboJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                if (msg->name[i] == JointParam_[j].name)
                {
                    orthrus_joint_state_msg_.motor[j].pos = msg->position[i];
                    orthrus_joint_state_msg_.motor[j].vec = msg->velocity[i];
                    orthrus_joint_state_msg_.motor[j].torq = msg->effort[i];
                }
            }
        }
        orthrus_joint_state_pub_->publish(orthrus_joint_state_msg_);
    }

    void orthrusGazeboNode::OrthrusGazeboImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        orthrus_imu_msg_.header = msg->header;                                                 // 复制头部信息
        orthrus_imu_msg_.orientation = msg->orientation;                                       // 复制方向四元数
        orthrus_imu_msg_.orientation_covariance = msg->orientation_covariance;                 // 复制方向协方差
        orthrus_imu_msg_.angular_velocity = msg->angular_velocity;                             // 复制角速度
        orthrus_imu_msg_.angular_velocity_covariance = msg->angular_velocity_covariance;       // 复制角速度协方差
        orthrus_imu_msg_.linear_acceleration = msg->linear_acceleration;                       // 复制线性加速度
        orthrus_imu_msg_.linear_acceleration_covariance = msg->linear_acceleration_covariance; // 复制线性加速度协方差
        orthrus_imu_pub_->publish(orthrus_imu_msg_);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_gazebo::orthrusGazeboNode>());
    rclcpp::shutdown();
    return 0;
}