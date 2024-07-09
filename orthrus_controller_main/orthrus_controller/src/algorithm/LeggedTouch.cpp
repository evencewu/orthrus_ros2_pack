#include "orthrus_controller/algorithm/LeggedTouch.hpp"

namespace orthrus_controller
{
    void LeggedTouch::Init(std::shared_ptr<OrthrusInterfaces> orthrus_interfaces_ptr,
                           std::shared_ptr<PinocchioInterfaces> pinocchio_ptr)
    {
        orthrus_interfaces_ = orthrus_interfaces_ptr;
        pinocchio_interfaces_ = pinocchio_ptr;
    }

    void LeggedTouch::Update()
    {
        /*
        // 定义关节配置和加速度
        Eigen::VectorXd q = pinocchio::neutral(model);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

        // 设定某个时间点的关节加速度和关节力矩
        // 这里用实际的关节加速度和力矩值替换
        a <<  joint accelerations ;
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
        tau <<  joint torques ;

        // 计算逆动力学以推断外力
        pinocchio::computeAllTerms(model, data, q, v);
        pinocchio::rnea(model, data, q, v, a); // 计算所有关节力矩

        // 计算末端执行器的雅可比矩阵
        Eigen::MatrixXd J = pinocchio::getFrameJacobian(model, data, model.getFrameId("end_effector_frame_name"), pinocchio::LOCAL_WORLD_ALIGNED);

        // 通过雅可比矩阵计算外力
        Eigen::VectorXd f_ext = J.transpose().inverse() * (tau - data.tau);

        // 输出外力
        std::cout << "External forces and torques: " << f_ext.transpose() << std::endl;
        */
    }
}