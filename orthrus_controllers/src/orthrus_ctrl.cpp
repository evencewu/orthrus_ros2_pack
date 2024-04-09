#include "orthrus_controllers/orthrus_ctrl.hpp"

namespace orthrus_ctrl
{
    orthrusCtrlNode::orthrusCtrlNode() : Node("orthrus_ctrl")
    {
        Init();

        orthrus_joint_state_sub_ = this->create_subscription<orthrus_interfaces::msg::OrthrusJointState>("/orthrus_interface/joint_state", 10, std::bind(&orthrusCtrlNode::OrthrusJointStateSubCallback, this, std::placeholders::_1));

        orthrus_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/orthrus_interface/imu", 10, std::bind(&orthrusCtrlNode::OrthrusImuSubCallback, this, std::placeholders::_1));

        orthrus_joint_control_pub_ = this->create_publisher<orthrus_interfaces::msg::OrthrusJointControl>("/orthrus_interface/joint_control", 10);

        orthrus_viewer_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/orthrus_viewer/joint_state", 10);

        orthrus_viewer_horizontal_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), std::bind(&orthrusCtrlNode::main_loop, this));
    }

    void orthrusCtrlNode::Init()
    {
        InitRobotParam();
    }

    void orthrusCtrlNode::main_loop()
    {
        UpdateRobotParma();
        orthrus_joint_control_msg_ = PositonCtrl_.StandUp();
        orthrus_joint_control_pub_->publish(orthrus_joint_control_msg_);
    }

    void orthrusCtrlNode::OrthrusJointStateSubCallback(const orthrus_interfaces::msg::OrthrusJointState::SharedPtr msg)
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

    void orthrusCtrlNode::OrthrusImuSubCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
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

    void orthrusCtrlNode::InitRobotParam()
    {
        RCLCPP_INFO(this->get_logger(), "run orthrus parma define\n");

        RCLCPP_INFO(this->get_logger(), "pinocchio urdf model loading\n");
        const std::string urdf_filename = std::string("/home/evence/code_file/ros2_ws/orthrus/src/orthrus_ros2_pack/orthrus_sim/orthrus_gazebo/models/orthrus/urdf/orthrus.urdf");

        pinocchio::urdf::buildModel(urdf_filename, orthrus_model_);
        RCLCPP_INFO(this->get_logger(), "over! model name: %s \n", orthrus_model_.name.c_str());

        // Create data required by the algorithms
        orthrus_data_ = pinocchio::Data(orthrus_model_);

        // Sample a random configuration
        OrthrusParam_.dynamic.joint_pos = randomConfiguration(orthrus_model_);

        q_ = Eigen::VectorXd::Zero(orthrus_model_.nq);
        v_ = Eigen::VectorXd::Zero(orthrus_model_.nv);

        // Perform the forward kinematics over the kinematic tree
        pinocchio::forwardKinematics(orthrus_model_, orthrus_data_, OrthrusParam_.dynamic.joint_pos);

        // Print out the placement of each joint of the kinematic tree
    }

    void orthrusCtrlNode::UpdateRobotParma()
    {
        // q_ = orthrus_data_.q;

        // slove and print joint coordinate
        for (int joint_id = 0; joint_id < 12; joint_id++)
        {
            OrthrusParam_.dynamic.joint_pos(joint_id) = OrthrusParam_.joint[joint_id].position;
        }

        pinocchio::forwardKinematics(orthrus_model_, orthrus_data_, OrthrusParam_.dynamic.joint_pos);
        pinocchio::updateGlobalPlacements(orthrus_model_, orthrus_data_);

#ifdef JOINT_INFO_LOG
        RCLCPP_INFO(this->get_logger(), "|   Joint Name   |         Link Position         | Joint Angle |");
        for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)orthrus_model_.njoints; ++joint_id)
        {
            RCLCPP_INFO(this->get_logger(), "| %-14s | %9lf %9lf %9lf | %11lf |",
                        orthrus_model_.names[joint_id].c_str(),
                        orthrus_data_.oMi[joint_id].translation()(0),
                        orthrus_data_.oMi[joint_id].translation()(1),
                        orthrus_data_.oMi[joint_id].translation()(2),
                        (joint_id > 0) ? OrthrusParam_.dynamic.joint_pos(joint_id - 1) : 0);
        }
#endif
        //std::ostringstream oss;
        //oss << orthrus_data_.C << std::endl;
        //std::string matrix_str = oss.str();
        //RCLCPP_INFO(this->get_logger(), "Matrix contents:\n%s", matrix_str.c_str());

        // slove and print joint coordinate
    }

    void orthrusCtrlNode::SolveLegKinematics()
    {
        /*
        Eigen::Vector3d target_position(0.1, 0.0, 0.0);
        Eigen::Quaterniond target_orientation(1, 0, 0, 0);

        //error slove
        Eigen::MatrixXd J = orthrus_data_.J[orthrus_model_.nframes - 1]; // 获取最后一个关节的雅可比矩阵
        Eigen::VectorXd dq = J.colPivHouseholderQr().solve(target_position - orthrus_data_.oMi[orthrus_model_.nframes - 1].translation()); // 求解关节期望角度增量
        Eigen::VectorXd q_desired = q_ + dq;

        orthrus_data_.q = q_desired;

        //q += dx;
        //data.q = q;

        //absolute slove

        //Eigen::Vector3d target_position(0.5, 0.5, 0.5); // 目标末端位置
        //Eigen::MatrixXd J = data.J[model.nframes - 1]; // 获取最后一个关节的雅可比矩阵
        //Eigen::VectorXd dq = J.colPivHouseholderQr().solve(target_position - data.oMi[model.nframes - 1].translation()); // 求解关节期望角度增量
        // 计算关节期望角度
        */
    }

    void orthrusCtrlNode::ResolveLegKinematics()
    {
    }

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_ctrl::orthrusCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
