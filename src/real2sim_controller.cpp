#include "real2sim_controller/real2sim_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace real2sim_controller {
Real2SimController::Real2SimController()
    : controller_interface::ControllerInterface(),
      rt_command_ptr_(nullptr),
      cmd_subscriber_(nullptr) {}

controller_interface::CallbackReturn Real2SimController::on_init() {
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Real2SimController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration Real2SimController::command_interface_configuration()
    const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::InterfaceConfiguration Real2SimController::state_interface_configuration()
    const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn Real2SimController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  // Populate the command interfaces map
  for (auto &command_interface : command_interfaces_) {
    command_interfaces_map_[command_interface.get_prefix_name()].emplace(
        command_interface.get_interface_name(), std::ref(command_interface));
  }

  // Populate the state interfaces map
  for (auto &state_interface : state_interfaces_) {
    state_interfaces_map_[state_interface.get_prefix_name()].emplace(
        state_interface.get_interface_name(), std::ref(state_interface));
  }

  // Store the initial joint positions
  for (int i = 0; i < ACTION_SIZE; i++) {
    init_joint_pos_[i] =
        state_interfaces_map_.at(params_.joint_names[i]).at("position").get().get_value();
  }

  init_time_ = get_node()->now();

  cmd_x_vel_ = 0.0;
  cmd_y_vel_ = 0.0;
  cmd_yaw_vel_ = 0.0;

  // Initialize the command subscriber
  cmd_subscriber_ = get_node()->create_subscription<CmdType>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Real2SimController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  for (auto &command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Real2SimController::update(const rclcpp::Time &time,
                                                             const rclcpp::Duration &period) {
  // When started, return to the default joint positions
  double time_since_init = (time - init_time_).seconds();
  if (time_since_init < params_.init_duration) {
    for (int i = 0; i < ACTION_SIZE; i++) {
      // Interpolate between the initial joint positions and the default joint
      // positions
      double interpolated_joint_pos =
          init_joint_pos_[i] * (1 - time_since_init / params_.init_duration) +
          params_.default_joint_pos[i] * (time_since_init / params_.init_duration);
      command_interfaces_map_.at(params_.joint_names[i])
          .at("position")
          .get()
          .set_value(interpolated_joint_pos);
      command_interfaces_map_.at(params_.joint_names[i])
          .at("kp")
          .get()
          .set_value(params_.init_kps[i]);
      command_interfaces_map_.at(params_.joint_names[i])
          .at("kd")
          .get()
          .set_value(params_.init_kds[i]);
    }
    return controller_interface::return_type::OK;
  }

  // After the init_duration has passed, fade in the policy actions
  double time_since_fade_in = (time - init_time_).seconds() - params_.init_duration;
  float fade_in_multiplier = std::min(time_since_fade_in / params_.fade_in_duration, 1.0);

  // If an emergency stop has been triggered, set all commands to 0 and return
  if (estop_active_) {
    for (auto &command_interface : command_interfaces_) {
      command_interface.set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Get the latest commanded velocities
  auto command = rt_command_ptr_.readFromRT();
  if (command && command->get()) {
    cmd_x_vel_ = command->get()->linear.x;
    cmd_y_vel_ = command->get()->linear.y;
    cmd_yaw_vel_ = command->get()->angular.z;
  }

  // Process the actions
  std::array<float, ACTION_SIZE> policy_output = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double freq = 1.0;
  double amp = 1.0;
  policy_output.at(2) = std::sin(time_since_fade_in * freq * M_PI) * amp;

  for (int i = 0; i < ACTION_SIZE; i++) {
    // Clip the action
    float action_clipped = std::max(std::min(policy_output[i], (float)params_.action_limit),
                                    (float)-params_.action_limit);
    // Scale and de-normalize to get the action vector
    if (params_.action_types[i] == "position") {
      action_[i] = fade_in_multiplier * action_clipped * params_.action_scales[i] +
                   params_.default_joint_pos[i];
    } else {
      action_[i] = fade_in_multiplier * action_clipped * params_.action_scales[i];
    }
    // Send the action to the hardware interface
    command_interfaces_map_.at(params_.joint_names[i])
        .at(params_.action_types[i])
        .get()
        .set_value((double)action_[i]);
    command_interfaces_map_.at(params_.joint_names[i]).at("kp").get().set_value(params_.kps[i]);
    command_interfaces_map_.at(params_.joint_names[i]).at("kd").get().set_value(params_.kds[i]);
  }

  return controller_interface::return_type::OK;
}

}  // namespace real2sim_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(real2sim_controller::Real2SimController,
                       controller_interface::ControllerInterface)