#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"

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
    auto logger = get_node()->get_logger();
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
    RCLCPP_INFO(logger, "Loading orthrus_interface");
    orthrus_interfaces_ = std::make_shared<OrthrusInterfaces>();
    RCLCPP_INFO(logger, "Loading visualization");
    visualization_ = std::make_shared<OrthrusVisualization>(get_node(), params_.leg_joint_names);
    RCLCPP_INFO(logger, "Loading visualization");
    calibration_visualization_ = std::make_shared<CalibrationVisualization>(get_node());
    RCLCPP_INFO(logger, "Loading pinocchio_interface");
    pinocchio_interfaces_ = std::make_shared<PinocchioInterfaces>(get_node());
    RCLCPP_INFO(logger, "Loading legged_odom");
    legged_odom_ = std::make_shared<LeggedOdom>(get_node());
    RCLCPP_INFO(logger, "Loading joy_interface");
    joy_interface_ = std::make_shared<JoyInterface>(get_node());
    RCLCPP_INFO(logger, "Loading legged_mpc");
    legged_mpc_ = std::make_shared<LeggedMpc>(get_node());
    RCLCPP_INFO(logger, "Loading safe_code");
    safe_code_ = std::make_shared<SafeCode>();

    params_.package_path = ament_index_cpp::get_package_share_directory(params_.package_name);
    RCLCPP_INFO(logger, "finish [%s]-path: %s", params_.package_name.c_str(), params_.package_path.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
  }

  InterfaceConfiguration OrthrusController::command_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    for (const auto &joint_name : params_.leg_joint_names)
    {
      conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    }

    if (params_.sim_or_real == "real")
    {
      conf_names.push_back("flag/enable_power");
      conf_names.push_back("flag/calibration_position");
      conf_names.push_back("flag/calibration_encoder");
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

    if (params_.sim_or_real == "real")
    {
      for (const auto &leg_imu_name : params_.leg_imu_names)
      {
        conf_names.push_back(leg_imu_name + "/" + "orientation.w");
        conf_names.push_back(leg_imu_name + "/" + "orientation.x");
        conf_names.push_back(leg_imu_name + "/" + "orientation.y");
        conf_names.push_back(leg_imu_name + "/" + "orientation.z");
      }
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

    RCLCPP_INFO(logger, "Init pinocchio_interfaces");
    pinocchio_interfaces_->Init(orthrus_interfaces_, params_.package_path);
    RCLCPP_INFO(logger, "Init visualization");
    visualization_->Init(orthrus_interfaces_);
    RCLCPP_INFO(logger, "Init legged_odom");
    legged_odom_->Init(orthrus_interfaces_);
    RCLCPP_INFO(logger, "Init joy_interfaces");
    joy_interface_->Init(orthrus_interfaces_);
    RCLCPP_INFO(logger, "Init legged_mpc");
    legged_mpc_->Init(orthrus_interfaces_, pinocchio_interfaces_);
    RCLCPP_INFO(logger, "Init safe_code");
    safe_code_->Init(orthrus_interfaces_);
    RCLCPP_INFO(logger, "Init calibration_visualization");
    calibration_visualization_->Init(orthrus_interfaces_);
    RCLCPP_INFO(logger, "Init over");

    RCLCPP_INFO(get_node()->get_logger(), "Init: \n%s", pinocchio_interfaces_->InitLogger().str().c_str());
    RCLCPP_INFO(logger, "Configure over...");

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OrthrusController::on_activate(
      const rclcpp_lifecycle::State &)
  {
    configure_joint(params_.leg_joint_names, joint_handles_);
    configure_imu(params_.imu_data_types, params_.imu_names, imu_handles_);
    if (params_.sim_or_real == "real")
    {
      configure_flag(params_.flag_data_types, params_.flag_names, flag_handles_);
      configure_leg_imu(params_.leg_imu_data_types, params_.leg_imu_names, leg_imu_handles_);
    }
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
    now_time_ = get_node()->now();
    last_time_ = now_time_;

    // orthrus_interfaces_->robot_target.gait_num = 0 ;

    rclcpp::Duration duration = period;

    auto logger = get_node()->get_logger();

    // Update joint data
    for (int joint_number = 0; joint_number < params_.leg_joint_num; joint_number++)
    {
      double position = joint_handles_[joint_number].state_position.get().get_value();
      double velocity = joint_handles_[joint_number].state_velocity.get().get_value();
      double effort = joint_handles_[joint_number].state_effort.get().get_value();
      orthrus_interfaces_->robot_state.joint.position[joint_number] = position;
      orthrus_interfaces_->robot_state.joint.velocity[joint_number] = velocity;
      orthrus_interfaces_->robot_state.joint.effort[joint_number] = effort;

      // if (std::isnan(position) || std::isnan(velocity) || std::isnan(effort))
      //{
      //   RCLCPP_ERROR(logger, "Either the joint is invalid for index");
      //   return controller_interface::return_type::ERROR;
      // }
    }

    // Update imu/odom data
    for (int i = 0; i < 3; i++)
    {
      orthrus_interfaces_->robot_state.body_imu.angular_velocity(i) = imu_handles_[0].angular_velocity[i].get().get_value();
    }

    for (int i = 0; i < 3; i++)
    {
      orthrus_interfaces_->robot_state.body_imu.linear_acceleration(i) = imu_handles_[0].linear_acceleration[i].get().get_value();
    }

    orthrus_interfaces_->robot_state.body_imu.orientation.w() = imu_handles_[0].orientation[0].get().get_value();
    orthrus_interfaces_->robot_state.body_imu.orientation.x() = imu_handles_[0].orientation[1].get().get_value();
    orthrus_interfaces_->robot_state.body_imu.orientation.y() = imu_handles_[0].orientation[2].get().get_value();
    orthrus_interfaces_->robot_state.body_imu.orientation.z() = imu_handles_[0].orientation[3].get().get_value();

    // RCLCPP_INFO(logger, "Joint position touch Limit %lf %lf %lf ",
    //             orthrus_interfaces_->odom_state.imu_position[0],
    //             orthrus_interfaces_->odom_state.imu_position[1],
    //             orthrus_interfaces_->odom_state.imu_position[2]);

    /*
    RCLCPP_INFO(logger, "target %lf %lf %lf %lf %lf %lf",
                orthrus_interfaces_->robot_target.target_position[0],
                orthrus_interfaces_->robot_target.target_position[1],
                orthrus_interfaces_->robot_target.target_position[2],
                orthrus_interfaces_->robot_target.target_euler[0],
                orthrus_interfaces_->robot_target.target_euler[1],
                orthrus_interfaces_->robot_target.target_euler[2]);

    RCLCPP_INFO(logger, "target %lf %lf %lf %lf %lf %lf",
                orthrus_interfaces_->robot_target.target_velocity[0],
                orthrus_interfaces_->robot_target.target_velocity[1],
                orthrus_interfaces_->robot_target.target_velocity[2],
                orthrus_interfaces_->robot_target.target_angular_velocity[0],
                orthrus_interfaces_->robot_target.target_angular_velocity[1],
                orthrus_interfaces_->robot_target.target_angular_velocity[2]);
    */
    RCLCPP_INFO(logger, "state %lf %lf %lf %lf %lf %lf",
                orthrus_interfaces_->odom_state.position[0],
                orthrus_interfaces_->odom_state.position[1],
                orthrus_interfaces_->odom_state.position[2],
                orthrus_interfaces_->odom_state.euler[0],
                orthrus_interfaces_->odom_state.euler[1],
                orthrus_interfaces_->odom_state.euler[2]);
    /*
    RCLCPP_INFO(logger, "odom_state.foot_position %lf %lf %lf",
                orthrus_interfaces_->odom_state.foot_position[0],
                orthrus_interfaces_->odom_state.foot_position[1],
                orthrus_interfaces_->odom_state.foot_position[2]);


    RCLCPP_INFO(logger, "robot_target.target_body_force %lf %lf %lf %lf %lf %lf",
                orthrus_interfaces_->robot_target.target_body_force[0],
                orthrus_interfaces_->robot_target.target_body_force[1],
                orthrus_interfaces_->robot_target.target_body_force[2],
                orthrus_interfaces_->robot_target.target_body_force[3],
                orthrus_interfaces_->robot_target.target_body_force[4],
                orthrus_interfaces_->robot_target.target_body_force[5]);
    */

    // orthrus_interfaces_->odom_state.touch_state[foot_num].touch_force[0]

    // Update leg_imu data
    if (params_.sim_or_real == "real")
    {
      for (int i = 0; i < 4; i++)
      {
        orthrus_interfaces_->robot_state.leg_imu[i].orientation.w() = leg_imu_handles_[i].orientation[0].get().get_value();
        orthrus_interfaces_->robot_state.leg_imu[i].orientation.x() = leg_imu_handles_[i].orientation[1].get().get_value();
        orthrus_interfaces_->robot_state.leg_imu[i].orientation.y() = leg_imu_handles_[i].orientation[2].get().get_value();
        orthrus_interfaces_->robot_state.leg_imu[i].orientation.z() = leg_imu_handles_[i].orientation[3].get().get_value();
      }
    }

    /*
    RCLCPP_INFO(logger, "Joint position touch Limit %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ", imu_handles_[0].angular_velocity[0].get().get_value(), imu_handles_[0].angular_velocity[1].get().get_value(), imu_handles_[0].angular_velocity[2].get().get_value(), imu_handles_[0].linear_acceleration[0].get().get_value(), imu_handles_[0].linear_acceleration[1].get().get_value(), imu_handles_[0].linear_acceleration[2].get().get_value(), imu_handles_[0].orientation[0].get().get_value(), imu_handles_[0].orientation[1].get().get_value(), imu_handles_[0].orientation[2].get().get_value(), imu_handles_[0].orientation[3].get().get_value());
    */

    /*
    RCLCPP_INFO(logger, "Joint position touch Limit %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf ",
                    leg_imu_handles_[0].orientation[0].get().get_value(),
                    leg_imu_handles_[0].orientation[1].get().get_value(),
                    leg_imu_handles_[0].orientation[2].get().get_value(),
                    leg_imu_handles_[0].orientation[3].get().get_value(),
                    leg_imu_handles_[1].orientation[0].get().get_value(),
                    leg_imu_handles_[1].orientation[1].get().get_value(),
                    leg_imu_handles_[1].orientation[2].get().get_value(),
                    leg_imu_handles_[1].orientation[3].get().get_value(),
                    leg_imu_handles_[2].orientation[0].get().get_value(),
                    leg_imu_handles_[2].orientation[1].get().get_value(),
                    leg_imu_handles_[2].orientation[2].get().get_value(),
                    leg_imu_handles_[2].orientation[3].get().get_value(),
                    leg_imu_handles_[3].orientation[0].get().get_value(),
                    leg_imu_handles_[3].orientation[1].get().get_value(),
                    leg_imu_handles_[3].orientation[2].get().get_value(),
                    leg_imu_handles_[3].orientation[3].get().get_value());*/

    // Update imu/odom data

    test_1_++;

    if (test_1_ >= 0 && test_1_ < 30)
    {
      // RCLCPP_INFO(logger, "OdomFilterLog: %s", legged_odom_->OdomFilterLog().str().c_str());
    }

    //----------------------------------------
    legged_odom_->Update(now_time_, duration);
    pinocchio_interfaces_->Update(time);
    legged_mpc_->Update(now_time_, duration);
    visualization_->Update(now_time_);

    if (params_.sim_or_real == "real")
    {
      calibration_visualization_->Update(now_time_);
    }

    // RCLCPP_INFO(logger, "data_.g %s", pinocchio_interfaces_->Logger().str().c_str());
    //----------------------------------------

    // orthrus_interfaces_->robot_cmd.effort[0] = 0.1;
    // orthrus_interfaces_->robot_cmd.effort[1] = 0.1;
    // orthrus_interfaces_->robot_cmd.effort[2] = 0.1;
    // orthrus_interfaces_->robot_cmd.effort[3] = -0.1;
    // orthrus_interfaces_->robot_cmd.effort[4] = 0.1;
    // orthrus_interfaces_->robot_cmd.effort[5] = 0.1;
    // orthrus_interfaces_->robot_cmd.effort[6] = -0.1;
    // orthrus_interfaces_->robot_cmd.effort[7] = -0.1;
    // orthrus_interfaces_->robot_cmd.effort[8] = -0.1;
    // orthrus_interfaces_->robot_cmd.effort[9] = 0.1;
    // orthrus_interfaces_->robot_cmd.effort[10] = -0.1;
    // orthrus_interfaces_->robot_cmd.effort[11] = -0.1;

    if (!safe_code_->PositionCheck())
    {
      RCLCPP_INFO(logger, "Joint position touch Limit");
    }

    std::stringstream ss;
    Eigen::VectorXd pos(6);
    pos << 0, 0, 25, 0, 0, 0;
    // ss << pinocchio_interfaces_->GetJacobianMatrix("LF_FOOT").block(0, 6, 6, 12).transpose()  * pos  << std::endl;
    // ss << pinocchio_interfaces_->GetJacobianMatrix("LF_FOOT") << std::endl;
    ss << pinocchio_interfaces_->GetJointEffortCompensation().transpose() << std::endl;
    RCLCPP_INFO(logger, "Matrix \n%s", ss.str().c_str());

    //"LH_FOOT"
    //"RF_FOOT"
    //"RH_FOOT"

    RCLCPP_INFO(logger, "effort %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                orthrus_interfaces_->robot_cmd.effort[0],
                orthrus_interfaces_->robot_cmd.effort[1],
                orthrus_interfaces_->robot_cmd.effort[2],
                orthrus_interfaces_->robot_cmd.effort[3],
                orthrus_interfaces_->robot_cmd.effort[4],
                orthrus_interfaces_->robot_cmd.effort[5],
                orthrus_interfaces_->robot_cmd.effort[6],
                orthrus_interfaces_->robot_cmd.effort[7],
                orthrus_interfaces_->robot_cmd.effort[8],
                orthrus_interfaces_->robot_cmd.effort[9],
                orthrus_interfaces_->robot_cmd.effort[10],
                orthrus_interfaces_->robot_cmd.effort[11]);

    std::stringstream ss_1;
    
    ss_1 << orthrus_interfaces_->odom_state.touch_state[0].touch_force << std::endl;
    RCLCPP_INFO(logger, "touch_force \n%s", ss_1.str().c_str());


    /*
    RCLCPP_INFO(logger, "position %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                orthrus_interfaces_->robot_state.joint.position[0],
                orthrus_interfaces_->robot_state.joint.position[1],
                orthrus_interfaces_->robot_state.joint.position[2],
                orthrus_interfaces_->robot_state.joint.position[3],
                orthrus_interfaces_->robot_state.joint.position[4],
                orthrus_interfaces_->robot_state.joint.position[5],
                orthrus_interfaces_->robot_state.joint.position[6],
                orthrus_interfaces_->robot_state.joint.position[7],
                orthrus_interfaces_->robot_state.joint.position[8],
                orthrus_interfaces_->robot_state.joint.position[9],
                orthrus_interfaces_->robot_state.joint.position[10],
                orthrus_interfaces_->robot_state.joint.position[11]);
    */

    if (orthrus_interfaces_->robot_target.if_enable)
    {
      RCLCPP_INFO(logger, "enable");
      for (int joint_number = 0; joint_number < params_.leg_joint_num; joint_number++)
      {
        joint_handles_[joint_number].cmd_effort.get().set_value(orthrus_interfaces_->robot_cmd.effort[joint_number]);
      }

      // joint_handles_[0].cmd_effort.get().set_value(0);
      // joint_handles_[3].cmd_effort.get().set_value(0);
      // joint_handles_[6].cmd_effort.get().set_value(0);
      // joint_handles_[9].cmd_effort.get().set_value(0);
    }
    else
    {
      for (int joint_number = 0; joint_number < params_.leg_joint_num; joint_number++)
      {
        joint_handles_[joint_number].cmd_effort.get().set_value(0.0);
      }
    }

    if (params_.sim_or_real == "real")
    {
      if (orthrus_interfaces_->robot_cmd.if_enable_power)
      {
        flag_handles_[0].enable_power.get().set_value(1.0);
      }
      else
      {
        flag_handles_[0].enable_power.get().set_value(0.0);
      }

      if (orthrus_interfaces_->robot_cmd.if_enable_calibration)
      {
        orthrus_interfaces_->robot_target.zero_yaw = orthrus_interfaces_->odom_state.euler[2];
        flag_handles_[0].calibration_position.get().set_value(1.0);
      }
      else
      {
        flag_handles_[0].calibration_position.get().set_value(0.0);
      }

      if (orthrus_interfaces_->robot_cmd.if_enable_calibration_encoder)
      {
        flag_handles_[0].calibration_encoder.get().set_value(1.0);
      }
      else
      {
        flag_handles_[0].calibration_encoder.get().set_value(0.0);
      }
    }

    return controller_interface::return_type::OK;
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

CLASS_LOADER_REGISTER_CLASS(orthrus_controller::OrthrusController, controller_interface::ControllerInterface)
