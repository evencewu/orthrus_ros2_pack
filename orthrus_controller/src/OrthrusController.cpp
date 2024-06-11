#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "orthrus_controller/OrthrusController.hpp"

namespace orthrus_controller
{
  using namespace std::chrono_literals;
  using controller_interface::interface_configuration_type;
  using controller_interface::InterfaceConfiguration;
  using lifecycle_msgs::msg::State;

  OrthrusController::OrthrusController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn OrthrusController::on_init()
  {
    RCLCPP_INFO(get_node()->get_logger(), "Loading orthrus controller...");
    try
    {
      // Create the parameter listener and get the parameters
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    // class
    RCLCPP_INFO(get_node()->get_logger(), "Loading visualization");
    visualization_ = std::make_shared<OrthrusVisualization>(get_node(), params_.leg_joint_names);
    RCLCPP_INFO(get_node()->get_logger(), "Loading pinocchio_interface");
    pinocchio_interface_ = std::make_shared<PinocchioInterface>(get_node());
    RCLCPP_INFO(get_node()->get_logger(), "Loading legged_odom");
    legged_odom_ = std::make_shared<LeggedOdom>(get_node());
    RCLCPP_INFO(get_node()->get_logger(), "Loading joy_interface");
    joy_interface_ = std::make_shared<JoyInterface>(get_node());
    RCLCPP_INFO(get_node()->get_logger(), "Loading legged_mpc");
    legged_mpc_ = std::make_shared<LeggedMpc>(get_node());

    // struct
    joint_state_ = std::make_shared<JointState>();
    odom_state_ = std::make_shared<OdomState>();
    touch_state_ = std::make_shared<std::vector<TouchState>>(4);
    mpc_target_ = std::make_shared<MpcTarget>();

    return controller_interface::CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration OrthrusController::command_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    for (const auto &joint_name : params_.leg_joint_names)
    {
      conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  InterfaceConfiguration OrthrusController::state_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    for (const auto &joint_name : params_.leg_joint_names)
    {
      conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
      conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }

    conf_names.push_back("imu_sensor/angular_velocity.x");
    conf_names.push_back("imu_sensor/angular_velocity.y");
    conf_names.push_back("imu_sensor/angular_velocity.z");
    conf_names.push_back("imu_sensor/linear_acceleration.x");
    conf_names.push_back("imu_sensor/linear_acceleration.y");
    conf_names.push_back("imu_sensor/linear_acceleration.z");
    conf_names.push_back("imu_sensor/orientation.w");
    conf_names.push_back("imu_sensor/orientation.x");
    conf_names.push_back("imu_sensor/orientation.y");
    conf_names.push_back("imu_sensor/orientation.z");

    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

  controller_interface::CallbackReturn OrthrusController::on_configure(
      const rclcpp_lifecycle::State &)
  {
    now_time_ = get_node()->now();
    last_time_ = now_time_;

    auto logger = get_node()->get_logger();
    RCLCPP_INFO(logger, "Configuring controller...");

    // update parameters if they have changed
    if (param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
      RCLCPP_INFO(logger, "Parameters were updated");
    }

    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(logger, "Init pinocchio_interface");
    pinocchio_interface_->Init(joint_state_, touch_state_);
    RCLCPP_INFO(logger, "Init visualization");
    visualization_->Init(joint_state_, odom_state_, touch_state_);
    RCLCPP_INFO(logger, "Init legged_odom");
    legged_odom_->Init(odom_state_, touch_state_);
    RCLCPP_INFO(logger, "Init joy_interface");
    //joy_interface_->Init(mpc_target_);
    RCLCPP_INFO(logger, "Init legged_mpc");
    legged_mpc_->Init(joint_state_, odom_state_, touch_state_, pinocchio_interface_);
    RCLCPP_INFO(logger, "Init over");

    RCLCPP_INFO(get_node()->get_logger(), "Init: \n%s", pinocchio_interface_->Logger().str().c_str());

    RCLCPP_INFO(logger, "Configure over...");

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OrthrusController::on_activate(
      const rclcpp_lifecycle::State &)
  {
    configure_joint(params_.leg_joint_names, joint_handles_);
    configure_imu(params_.imu_data_types, params_.imu_names, imu_handles_);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OrthrusController::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    /*
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    received_velocity_msg_ptr_.set(std::make_shared<Twist>());
    */
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OrthrusController::on_error(
      const rclcpp_lifecycle::State &)
  {
    if (!reset())
    {
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OrthrusController::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    /*
    subscriber_is_active_ = false;
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    */
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OrthrusController::on_shutdown(
      const rclcpp_lifecycle::State &)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type OrthrusController::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {

    // mpc_target_->;

    now_time_ = get_node()->now();
    last_time_ = now_time_;

    rclcpp::Duration duration = now_time_ - last_time_;

    auto logger = get_node()->get_logger();

    // Update joint data
    for (int joint_number = 0; joint_number < params_.leg_joint_num; joint_number++)
    {
      double position = joint_handles_[joint_number].state_position.get().get_value();
      double velocity = joint_handles_[joint_number].state_velocity.get().get_value();
      double effort = joint_handles_[joint_number].state_effort.get().get_value();
      joint_state_->position[joint_number] = position;
      joint_state_->velocity[joint_number] = velocity;
      joint_state_->effort[joint_number] = effort;

      if (std::isnan(position) || std::isnan(velocity) || std::isnan(effort))
      {
        RCLCPP_ERROR(logger, "Either the joint is invalid for index");
        return controller_interface::return_type::ERROR;
      }
    }

    // Update imu/odom data
    for (int i = 0; i < 3; i++)
    {
      odom_state_->imu.angular_velocity(i) = imu_handles_[0].angular_velocity[i].get().get_value();
    }

    for (int i = 0; i < 3; i++)
    {
      odom_state_->imu.linear_acceleration(i) = imu_handles_[0].linear_acceleration[i].get().get_value();
    }

    odom_state_->imu.orientation.w() = imu_handles_[0].orientation[0].get().get_value();
    odom_state_->imu.orientation.x() = imu_handles_[0].orientation[1].get().get_value();
    odom_state_->imu.orientation.y() = imu_handles_[0].orientation[2].get().get_value();
    odom_state_->imu.orientation.z() = imu_handles_[0].orientation[3].get().get_value();

    visualization_->Update(now_time_);
    legged_odom_->Update(now_time_, duration);
    pinocchio_interface_->Update(now_time_);
    legged_mpc_->Update(now_time_, duration);

    //Eigen::VectorXd acc = pinocchio_interface_->LegGravityCompensation();

    //Eigen::MatrixXd jac_1 = pinocchio_interface_->GetJacobianMatrix("LF_FOOT");
    //Eigen::MatrixXd jac_2 = pinocchio_interface_->GetJacobianMatrix("LH_FOOT");
    //Eigen::MatrixXd jac_3 = pinocchio_interface_->GetJacobianMatrix("RF_FOOT");
    //Eigen::MatrixXd jac_4 = pinocchio_interface_->GetJacobianMatrix("RH_FOOT");

    std::stringstream ss;
    //ss << "acc:" << acc.transpose() << std::endl;
    //Eigen::Matrix<double, 6, 1> F;
    //F <<  1.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 示例力向量

    //ss << "jac:\n" << jac_1.transpose() * F<< std::endl;
    //RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());
    //ss << "jac:\n" << jac_2.transpose()<< std::endl;
    //RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());
    //ss << "jac:\n" << jac_3.transpose()<< std::endl;
    //ss << "jac:\n" << jac_4.transpose()<< std::endl;


    

    // RCLCPP_INFO(get_node()->get_logger(), "position\n%s", pinocchio_interface_->LegPositionInterpolation().str().c_str());

    // RCLCPP_INFO(logger, "imu: %lf", imu_handles_[0].orientation[0].get().get_value());

    // RCLCPP_INFO(get_node()->get_logger(), "JOINT\n%s", pinocchio_interface_->Logger().str().c_str());

    return controller_interface::return_type::OK;
  }

  controller_interface::CallbackReturn OrthrusController::configure_joint(
      const std::vector<std::string> &joint_names,
      std::vector<JointHandle> &registered_handles)
  {
    auto logger = get_node()->get_logger();

    if (joint_names.empty())
    {
      RCLCPP_ERROR(logger, "No motor names specified");
      return controller_interface::CallbackReturn::ERROR;
    }

    // register handles
    registered_handles.reserve(joint_names.size());
    for (const auto &joint_name : joint_names)
    {
      const auto position_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&joint_name](const auto &interface)
          {
            return interface.get_prefix_name() == joint_name &&
                   interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
          });

      if (position_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto velocity_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&joint_name](const auto &interface)
          {
            return interface.get_prefix_name() == joint_name &&
                   interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
          });

      if (velocity_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto effort_handle = std::find_if(
          state_interfaces_.cbegin(), state_interfaces_.cend(),
          [&joint_name](const auto &interface)
          {
            return interface.get_prefix_name() == joint_name &&
                   interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
          });

      if (effort_handle == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      const auto command_handle = std::find_if(
          command_interfaces_.begin(), command_interfaces_.end(),
          [&joint_name](const auto &interface)
          {
            return interface.get_prefix_name() == joint_name &&
                   interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
          });

      if (command_handle == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "Unable to obtain motor command handle for %s", joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }

      registered_handles.emplace_back(
          JointHandle{std::ref(*position_handle), std::ref(*velocity_handle), std::ref(*effort_handle), std::ref(*command_handle)});
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OrthrusController::configure_imu(
      const std::vector<std::string> &imu_data_types,
      const std::vector<std::string> &imu_names,
      std::vector<ImuHandle> &registered_handles)
  {
    auto logger = get_node()->get_logger();

    if (imu_names.empty())
    {
      RCLCPP_ERROR(logger, "No motor names specified");
      return controller_interface::CallbackReturn::ERROR;
    }

    // register handles
    registered_handles.reserve(imu_names.size());

    for (const auto &imu_name : imu_names)
    {
      std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> angular_velocity;
      std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> linear_acceleration;
      std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> orientation;

      for (const auto &imu_data_type : imu_data_types)
      {
        const auto imu_handle = std::find_if(
            state_interfaces_.cbegin(), state_interfaces_.cend(),
            [&imu_name, &imu_data_type](const auto &interface)
            {
              return interface.get_prefix_name() == imu_name &&
                     interface.get_interface_name() == imu_data_type;
            });

        if (imu_handle == state_interfaces_.cend())
        {
          RCLCPP_ERROR(logger, "Unable to obtain motor state handle for %s", imu_name.c_str());
          return controller_interface::CallbackReturn::ERROR;
        }

        if (imu_data_type == "angular_velocity.x" || imu_data_type == "angular_velocity.y" || imu_data_type == "angular_velocity.z")
        {
          // RCLCPP_INFO(get_node()->get_logger(), "imu %lf", *imu_handle.get().get_value());
          angular_velocity.emplace_back(std::ref(*imu_handle));
        }

        if (imu_data_type == "linear_acceleration.x" || imu_data_type == "linear_acceleration.y" || imu_data_type == "linear_acceleration.z")
        {
          linear_acceleration.emplace_back(std::ref(*imu_handle));
        }

        if (imu_data_type == "orientation.w" || imu_data_type == "orientation.x" || imu_data_type == "orientation.y" || imu_data_type == "orientation.z")
        {
          orientation.emplace_back(std::ref(*imu_handle));
        }
      }
      registered_handles.emplace_back(ImuHandle{angular_velocity, linear_acceleration, orientation});
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool OrthrusController::reset()
  {
    /*
    // release the old queue
    std::queue<Twist> empty;
    std::swap(previous_commands_, empty);

    registered_left_wheel_handles_.clear();
    registered_right_wheel_handles_.clear();

    subscriber_is_active_ = false;
    velocity_command_subscriber_.reset();
    received_velocity_msg_ptr_.set(nullptr);
    */

    is_halted = false;
    return true;
  }

  void OrthrusController::halt()
  {
    // make wheels stop
    /*
    const auto halt_wheels = [](auto &wheel_handles)
    {
      for (const auto &wheel_handle : wheel_handles)
      {
        wheel_handle.velocity.get().set_value(0.0);
      }
    };

    halt_wheels(registered_left_wheel_handles_);
    halt_wheels(registered_right_wheel_handles_);
    */
  }

} // namespace diff_test_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    orthrus_controller::OrthrusController, controller_interface::ControllerInterface)
