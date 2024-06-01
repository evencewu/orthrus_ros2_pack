#include "orthrus_controller/interfaces/PinocchioInterface.hpp"

namespace orthrus_controller
{
    void PinocchioInterface(const rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    {

    }

    void PinocchioInterface::Init(const rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    {
        pinocchio::buildModels::manipulator(model_);
        pinocchio::Data data(model_);

        Eigen::VectorXd q = pinocchio::neutral(model_);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);

        const Eigen::VectorXd &tau = pinocchio::rnea(model_, data, q, v, a);
        std::cout << "tau = " << tau.transpose() << std::endl;
    }

}
