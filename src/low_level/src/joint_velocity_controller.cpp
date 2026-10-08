#include "low_level/joint_velocity_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace low_level {
namespace {

template <std::size_t N>
bool copy_parameter_array(const std::vector<double>& input, std::array<double, N>& output) {
  if (input.size() != N) {
    return false;
  }
  std::copy(input.begin(), input.end(), output.begin());
  return true;
}

template <std::size_t N>
bool all_finite_nonnegative(const std::array<double, N>& values) {
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value) && value >= 0.0;
  });
}

}  // namespace

controller_interface::InterfaceConfiguration
JointVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return config;
}

JointVelocityController::CallbackReturn JointVelocityController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::vector<std::string>>("joint_names", {});
    auto_declare<std::string>("command_topic", "joint_commands");
    auto_declare<double>("command_timeout", 0.1);
    auto_declare<double>("velocity_scale", 1.0);
    auto_declare<double>("tracking_time_constant", 0.03);
    auto_declare<std::vector<double>>("max_velocity", std::vector<double>(kNumJoints, 0.0));
    auto_declare<std::vector<double>>("max_acceleration", std::vector<double>(kNumJoints, 2.0));
    auto_declare<std::vector<double>>("max_jerk", std::vector<double>(kNumJoints, 100.0));
    auto_declare<std::string>("controller_version", "velocity_tracker_v5");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

JointVelocityController::CallbackReturn JointVelocityController::on_configure(
    const rclcpp_lifecycle::State&) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  command_timeout_s_ = get_node()->get_parameter("command_timeout").as_double();
  velocity_scale_ = get_node()->get_parameter("velocity_scale").as_double();
  tracking_time_constant_s_ = get_node()->get_parameter("tracking_time_constant").as_double();

  if (joint_names_.empty()) {
    joint_names_ = derive_joint_names();
  }
  if (joint_names_.size() != kNumJoints) {
    RCLCPP_ERROR(get_node()->get_logger(), "joint_names must contain exactly 7 names");
    return CallbackReturn::ERROR;
  }
  if (command_timeout_s_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "command_timeout must be > 0");
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(velocity_scale_) || velocity_scale_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "velocity_scale must be finite and >= 0");
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(tracking_time_constant_s_) || tracking_time_constant_s_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "tracking_time_constant must be finite and > 0");
    return CallbackReturn::ERROR;
  }

  if (!copy_parameter_array(get_node()->get_parameter("max_velocity").as_double_array(), max_velocity_) ||
      !copy_parameter_array(get_node()->get_parameter("max_acceleration").as_double_array(), max_acceleration_) ||
      !copy_parameter_array(get_node()->get_parameter("max_jerk").as_double_array(), max_jerk_)) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "max_velocity, max_acceleration, and max_jerk must each contain exactly 7 values");
    return CallbackReturn::ERROR;
  }

  if (!all_finite_nonnegative(max_velocity_) ||
      !all_finite_nonnegative(max_acceleration_) ||
      !all_finite_nonnegative(max_jerk_)) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "max_velocity, max_acceleration, and max_jerk values must be finite and >= 0");
    return CallbackReturn::ERROR;
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (max_acceleration_[i] <= 0.0 || max_jerk_[i] <= 0.0) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "max_acceleration and max_jerk must be > 0 for every joint");
      return CallbackReturn::ERROR;
    }
  }

  VelocityCommand initial;
  command_buffer_.writeFromNonRT(initial);
  last_velocity_command_.fill(0.0);
  last_acceleration_command_.fill(0.0);

  command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      command_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
      std::bind(&JointVelocityController::command_callback, this, std::placeholders::_1));
  ready_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/ready", rclcpp::QoS(1).transient_local().reliable());

  const auto version = get_node()->get_parameter("controller_version").as_string();
  RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured velocity controller [%s] on topic '%s'. "
      "Using damped direction-preserving 1 kHz velocity tracking. "
      "tracking_time_constant=%.4f s, "
      "max_acceleration=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f], "
      "max_jerk=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
      version.c_str(), command_topic_.c_str(), tracking_time_constant_s_,
      max_acceleration_[0], max_acceleration_[1], max_acceleration_[2],
      max_acceleration_[3], max_acceleration_[4], max_acceleration_[5],
      max_acceleration_[6], max_jerk_[0], max_jerk_[1], max_jerk_[2],
      max_jerk_[3], max_jerk_[4], max_jerk_[5], max_jerk_[6]);
  return CallbackReturn::SUCCESS;
}

JointVelocityController::CallbackReturn JointVelocityController::on_activate(
    const rclcpp_lifecycle::State&) {
  last_velocity_command_.fill(0.0);
  last_acceleration_command_.fill(0.0);
  for (auto& interface : command_interfaces_) {
    if (!interface.set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize velocity command interface");
      return CallbackReturn::ERROR;
    }
  }
  if (ready_pub_) {
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);
  }
  RCLCPP_INFO(get_node()->get_logger(), "Velocity controller is operational");
  return CallbackReturn::SUCCESS;
}

JointVelocityController::CallbackReturn JointVelocityController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  last_velocity_command_.fill(0.0);
  last_acceleration_command_.fill(0.0);
  for (auto& interface : command_interfaces_) {
    if (!interface.set_value(0.0)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to zero velocity command interface");
      return CallbackReturn::ERROR;
    }
  }
  if (ready_pub_) {
    std_msgs::msg::Bool ready;
    ready.data = false;
    ready_pub_->publish(ready);
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointVelocityController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period) {
  (void)time;
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  const auto* command = command_buffer_.readFromRT();
  const auto steady_now = std::chrono::steady_clock::now();
  const bool timed_out =
      command == nullptr || !command->valid ||
      std::chrono::duration<double>(steady_now - command->received_at).count() > command_timeout_s_;

  std::array<double, kNumJoints> target{};
  if (!timed_out) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      target[i] = command->velocity[i] * velocity_scale_;
    }

    // Final optional velocity guard. Scale the complete vector instead of
    // clipping individual joints, preserving the requested joint-space direction.
    double worst_velocity_ratio = 1.0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (max_velocity_[i] <= 0.0) {
        continue;
      }
      worst_velocity_ratio =
          std::max(worst_velocity_ratio, std::abs(target[i]) / max_velocity_[i]);
    }
    if (worst_velocity_ratio > 1.0) {
      for (double& value : target) {
        value /= worst_velocity_ratio;
      }
    }
  }

  // The high-level IK/QP runs at a lower rate (typically 200 Hz), while the
  // Franka velocity interface is updated at 1 kHz. Direct passthrough creates
  // a velocity step at each high-level update and can trigger Franka jerk
  // discontinuity reflexes. The previous v4 tracker used (target-v)/dt, which
  // is effectively an extremely high-gain velocity servo (gain ~1000 1/s) and
  // made small IK command ripple audible at the motors.
  //
  // Track the requested velocity with a finite time constant instead:
  //        a_des = (v_target - v_command) / tau
  // then limit the complete acceleration and jerk vectors uniformly. This is a
  // monotone first-order velocity target before jerk shaping, preserves joint
  // ratios, and provides the required 1 kHz continuity without the old slow
  // jerk=10 behavior or the noisy v4 jerk=1000 behavior.
  std::array<double, kNumJoints> desired_acceleration{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    desired_acceleration[i] =
        (target[i] - last_velocity_command_[i]) / tracking_time_constant_s_;
  }

  double worst_acceleration_ratio = 1.0;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    worst_acceleration_ratio = std::max(
        worst_acceleration_ratio,
        std::abs(desired_acceleration[i]) / max_acceleration_[i]);
  }
  if (worst_acceleration_ratio > 1.0) {
    for (double& value : desired_acceleration) {
      value /= worst_acceleration_ratio;
    }
  }

  std::array<double, kNumJoints> acceleration_delta{};
  double worst_jerk_ratio = 1.0;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    acceleration_delta[i] = desired_acceleration[i] - last_acceleration_command_[i];
    const double max_delta = max_jerk_[i] * dt;
    worst_jerk_ratio =
        std::max(worst_jerk_ratio, std::abs(acceleration_delta[i]) / max_delta);
  }
  if (worst_jerk_ratio > 1.0) {
    for (double& value : acceleration_delta) {
      value /= worst_jerk_ratio;
    }
  }

  std::array<double, kNumJoints> acceleration{};
  std::array<double, kNumJoints> velocity{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    acceleration[i] = last_acceleration_command_[i] + acceleration_delta[i];
    velocity[i] = last_velocity_command_[i] + acceleration[i] * dt;
  }

  // Never snap velocity or acceleration to the target: Franka differentiates
  // the 1 kHz command, so a snap would recreate an acceleration/jerk discontinuity.
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!command_interfaces_[i].set_value(velocity[i])) {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Failed to write velocity command for joint %zu", i);
      return controller_interface::return_type::ERROR;
    }
  }

  last_velocity_command_ = velocity;
  last_acceleration_command_ = acceleration;
  return controller_interface::return_type::OK;
}

void JointVelocityController::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  VelocityCommand command;
  if (!extract_velocity(*msg, command.velocity)) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Ignoring malformed JointState command: velocity must address all 7 joints");
    return;
  }

  for (const double velocity : command.velocity) {
    if (!std::isfinite(velocity)) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "Ignoring JointState command containing a non-finite velocity");
      return;
    }
  }

  command.received_at = std::chrono::steady_clock::now();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

bool JointVelocityController::extract_velocity(
    const sensor_msgs::msg::JointState& msg,
    std::array<double, kNumJoints>& velocity) const {
  if (msg.name.empty()) {
    if (msg.velocity.size() != kNumJoints) {
      return false;
    }
    std::copy(msg.velocity.begin(), msg.velocity.end(), velocity.begin());
    return true;
  }
  if (msg.velocity.size() != msg.name.size()) {
    return false;
  }

  // Avoid constructing an unordered_map for every 200 Hz command message.
  // This callback is non-RT, but allocator activity can still create CPU/cache
  // jitter on the same host as the 1 kHz Franka control thread. Seven joints
  // are small enough that a fixed bounded search is cheaper and allocation-free.
  for (std::size_t joint_i = 0; joint_i < kNumJoints; ++joint_i) {
    bool found = false;
    for (std::size_t msg_i = 0; msg_i < msg.name.size(); ++msg_i) {
      if (msg.name[msg_i] == joint_names_[joint_i]) {
        velocity[joint_i] = msg.velocity[msg_i];
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

std::vector<std::string> JointVelocityController::derive_joint_names() const {
  std::vector<std::string> names;
  names.reserve(kNumJoints);
  const std::string prefix = (arm_prefix_.empty() ? std::string{} : arm_prefix_ + "_") + robot_type_;
  for (std::size_t i = 1; i <= kNumJoints; ++i) {
    names.push_back(prefix + "_joint" + std::to_string(i));
  }
  return names;
}

}  // namespace low_level

PLUGINLIB_EXPORT_CLASS(low_level::JointVelocityController, controller_interface::ControllerInterface)
