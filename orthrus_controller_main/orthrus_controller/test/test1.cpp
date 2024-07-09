#include <iostream>

#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include <Eigen/Dense>

int main()
{
  std::string path_name = "/home/orthrus/orthrus/src/orthrus_ros2_pack/orthrus_interfaces";
  std::string urdf_filename_ = "/models/orthrus/urdf/orthrus.urdf";

  std::string urdf_path_ = path_name + urdf_filename_;

  pinocchio::Model model_;

  pinocchio::Data data_;

  Eigen::VectorXd joint_position_;
  Eigen::VectorXd joint_velocity_;
  Eigen::VectorXd joint_acceleration_;

  pinocchio::urdf::buildModel(urdf_path_, pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);

  joint_position_ = pinocchio::neutral(model_);
  joint_velocity_ = Eigen::VectorXd::Zero(model_.nv);     // 速度为零
  joint_acceleration_ = Eigen::VectorXd::Zero(model_.nv); // 关节加速度

  pinocchio::forwardKinematics(model_, data_, joint_position_);
  pinocchio::framesForwardKinematics(model_, data_, joint_position_);
  pinocchio::updateGlobalPlacements(model_, data_);

  // 广义重力项计算
  pinocchio::computeGeneralizedGravity(model_, data_, joint_position_);
  pinocchio::nonLinearEffects(model_, data_, joint_position_, joint_velocity_);
  pinocchio::computeCoriolisMatrix(model_, data_, joint_position_, joint_velocity_);

  // 雅可比矩阵计算
  pinocchio::computeJointJacobians(model_, data_, joint_position_);

  Eigen::VectorXd target_acceleration = Eigen::VectorXd::Zero(model_.nv); // 关节加速度
  joint_position_[2] = 10;
  target_acceleration[2] = -9.81;

  const Eigen::VectorXd &tau = pinocchio::rnea(model_, data_, joint_position_, joint_velocity_, target_acceleration);
  std::cout << "tau = " << tau.transpose() << std::endl;

  std::stringstream ss;

  ss << "model_.nv: " << model_.nv << std::endl;

  for (size_t i = 0; i < model_.frames.size(); ++i)
  {
    const pinocchio::Frame &frame = model_.frames[i];
    ss << "Frame " << i << ": " << std::endl;
    ss << "  Name: " << frame.name << std::endl;
    ss << "  Parent joint ID: " << frame.parent << std::endl;
    ss << "  Frame type: " << frame.type << std::endl;
  }

  for (int joint_id = 1; joint_id < 14; joint_id++)
  {
    ss << joint_id << " " << model_.names[joint_id] << std::endl;
  }

  for (int joint_id = 0; joint_id < 19; joint_id++)
  {
    ss << joint_id << " " << joint_position_[joint_id] << std::endl;
  }

  ss << "data_.g" << data_.g << std::endl;
  ss << "data_.nle" << data_.nle << std::endl;
  ss << "data_.C" << data_.C << std::endl;

  std::cout << ss.str().c_str() << std::endl;
}