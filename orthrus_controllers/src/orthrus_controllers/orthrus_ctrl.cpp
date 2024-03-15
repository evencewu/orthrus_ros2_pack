#include "orthrus_controllers/orthrus_ctrl.hpp"

namespace orthrus_ctrl
{
    orthrusCtrlNode::orthrusCtrlNode() : Node("orthrus_ctrl")
    {
        init();

        orthrus_joint_state_sub_ = this->create_subscription<orthrus_interfaces::msg::OrthrusJointState>("/orthrus_interface/joint_state", 10, std::bind(&orthrusCtrlNode::OrthrusJointStateSubCallback, this, std::placeholders::_1));

        orthrus_joint_control_pub_ = this->create_publisher<orthrus_interfaces::msg::OrthrusJointControl>("/orthrus_interface/joint_control", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), std::bind(&orthrusCtrlNode::main_loop, this));
    }

    void orthrusCtrlNode::init()
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
        for (int i = 0; i < 12; i++)
        {
            OrthrusParam_.joint[i].position = msg->motor[i].pos;
            OrthrusParam_.joint[i].velocity = msg->motor[i].vec;
            OrthrusParam_.joint[i].effort = msg->motor[i].torq;
        }
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

        const Eigen::MatrixXd& M = orthrus_data_.M; // 惯性矩阵
        const Eigen::MatrixXd& C = orthrus_data_.C; // 科里奥利和向心力矩阵
        const Eigen::MatrixXd& g = orthrus_data_.g; // 重力项（不是完整的G矩阵，但通常与G矩阵一起使用）

        // Perform the forward kinematics over the kinematic tree
        forwardKinematics(orthrus_model_, orthrus_data_, OrthrusParam_.dynamic.joint_pos);

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

        forwardKinematics(orthrus_model_, orthrus_data_, OrthrusParam_.dynamic.joint_pos);

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

        //

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
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<orthrus_ctrl::orthrusCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
