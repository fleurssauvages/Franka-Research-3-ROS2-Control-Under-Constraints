#include "low_level/joint_impedance_controller.hpp"

#include <algorithm>
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
bool copy_enable_mask(const std::vector<double>& input, std::array<bool, N>& output) {
  if (input.size() != N) {
    return false;
  }
  for (std::size_t i = 0; i < N; ++i) {
    if (!std::isfinite(input[i])) {
      return false;
    }
    output[i] = input[i] > 0.5;
  }
  return true;
}

}  // namespace

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  if (use_mass_damping_ && franka_robot_model_) {
    const auto model_interfaces = franka_robot_model_->get_state_interface_names();
    config.names.insert(config.names.end(), model_interfaces.begin(), model_interfaces.end());
  }
  return config;
}

JointImpedanceController::CallbackReturn JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::vector<std::string>>("joint_names", {});
    auto_declare<std::string>("command_topic", "joint_commands");
    auto_declare<double>("command_timeout", 0.01);
    auto_declare<std::vector<double>>("stiffness", std::vector<double>(kNumJoints, 0.0));
    auto_declare<std::vector<double>>("damping", {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5});
    auto_declare<std::vector<double>>("mass_damping", {3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0});
    auto_declare<double>("effort_feedforward_scale", 1.0);
    auto_declare<double>("delta_tau_max", 1.0);
    auto_declare<std::vector<double>>("max_torque", std::vector<double>(kNumJoints, 0.0));
    auto_declare<bool>("hold_position_on_timeout", false);

    auto_declare<bool>("friction_compensation_enabled", true);
    auto_declare<bool>("friction_calibration_valid", false);
    auto_declare<std::string>("friction_calibration_source", "");
    auto_declare<double>("friction_calibration_age_hours", -1.0);
    auto_declare<double>("friction_compensation_scale", 0.3);
    auto_declare<double>("friction_smoothing_velocity", 0.01);
    auto_declare<bool>("friction_stribeck_enabled", true);
    auto_declare<std::vector<double>>(
        "friction_stribeck_velocity_per_joint", std::vector<double>(kNumJoints, 0.05));
    auto_declare<std::vector<double>>("friction_static", std::vector<double>(kNumJoints, 0.0));
    auto_declare<std::vector<double>>(
        "friction_max_compensation_torque", std::vector<double>(kNumJoints, 0.75));
    auto_declare<double>("friction_validated_velocity_min", 0.02);
    auto_declare<std::vector<double>>(
        "friction_validated_velocity_min_per_joint", std::vector<double>(kNumJoints, 0.02));
    // Keep the legacy scalar parameter for compatibility with previously built binaries/launch files.
    auto_declare<double>("friction_validated_velocity_max", 0.25);
    auto_declare<std::vector<double>>(
        "friction_validated_velocity_max_per_joint",
        std::vector<double>(kNumJoints, 0.25));
    auto_declare<std::vector<double>>("friction_joint_enable", std::vector<double>(kNumJoints, 0.0));
    auto_declare<std::vector<double>>("friction_coulomb", std::vector<double>(kNumJoints, 0.0));
    auto_declare<std::vector<double>>("friction_viscous", std::vector<double>(kNumJoints, 0.0));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

JointImpedanceController::CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State&) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  command_timeout_s_ = get_node()->get_parameter("command_timeout").as_double();
  effort_feedforward_scale_ = get_node()->get_parameter("effort_feedforward_scale").as_double();
  delta_tau_max_ = get_node()->get_parameter("delta_tau_max").as_double();
  hold_position_on_timeout_ = get_node()->get_parameter("hold_position_on_timeout").as_bool();

  friction_compensation_requested_ =
      get_node()->get_parameter("friction_compensation_enabled").as_bool();
  friction_calibration_valid_ =
      get_node()->get_parameter("friction_calibration_valid").as_bool();
  friction_calibration_source_ =
      get_node()->get_parameter("friction_calibration_source").as_string();
  friction_calibration_age_hours_ =
      get_node()->get_parameter("friction_calibration_age_hours").as_double();
  friction_compensation_scale_ =
      get_node()->get_parameter("friction_compensation_scale").as_double();
  friction_smoothing_velocity_ =
      get_node()->get_parameter("friction_smoothing_velocity").as_double();
  friction_stribeck_enabled_ = get_node()->get_parameter("friction_stribeck_enabled").as_bool();
  friction_validated_velocity_min_ =
      get_node()->get_parameter("friction_validated_velocity_min").as_double();

  if (joint_names_.empty()) {
    joint_names_ = derive_joint_names();
  }
  if (joint_names_.size() != kNumJoints) {
    RCLCPP_ERROR(get_node()->get_logger(), "joint_names must contain exactly 7 names");
    return CallbackReturn::ERROR;
  }
  if (command_timeout_s_ <= 0.0 || delta_tau_max_ < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "command_timeout must be > 0 and delta_tau_max >= 0");
    return CallbackReturn::ERROR;
  }

  if (!copy_parameter_array(get_node()->get_parameter("stiffness").as_double_array(), stiffness_) ||
      !copy_parameter_array(get_node()->get_parameter("damping").as_double_array(), damping_) ||
      !copy_parameter_array(get_node()->get_parameter("mass_damping").as_double_array(), mass_damping_) ||
      !copy_parameter_array(get_node()->get_parameter("max_torque").as_double_array(), max_torque_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "stiffness, damping, mass_damping, and max_torque must each contain exactly 7 values");
    return CallbackReturn::ERROR;
  }

  if (!copy_parameter_array(
          get_node()->get_parameter("friction_static").as_double_array(), friction_static_) ||
      !copy_parameter_array(
          get_node()->get_parameter("friction_stribeck_velocity_per_joint").as_double_array(),
          friction_stribeck_velocity_) ||
      !copy_parameter_array(
          get_node()->get_parameter("friction_max_compensation_torque").as_double_array(),
          friction_max_compensation_torque_) ||
      !copy_parameter_array(
          get_node()->get_parameter("friction_validated_velocity_min_per_joint").as_double_array(),
          friction_validated_velocity_min_per_joint_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "friction_static, Stribeck speeds, validated minimum speeds and torque limits must have seven values");
    return CallbackReturn::ERROR;
  }

  if (!copy_enable_mask(
          get_node()->get_parameter("friction_joint_enable").as_double_array(),
          friction_joint_enable_) ||
      !copy_parameter_array(
          get_node()->get_parameter("friction_coulomb").as_double_array(),
          friction_coulomb_) ||
      !copy_parameter_array(
          get_node()->get_parameter("friction_viscous").as_double_array(),
          friction_viscous_) ||
      !copy_parameter_array(
          get_node()->get_parameter("friction_validated_velocity_max_per_joint").as_double_array(),
          friction_validated_velocity_max_)) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "friction_joint_enable, friction_coulomb, friction_viscous, and "
        "friction_validated_velocity_max_per_joint must contain exactly 7 values");
    return CallbackReturn::ERROR;
  }

  if (!std::isfinite(friction_compensation_scale_) || friction_compensation_scale_ < 0.0 ||
      !std::isfinite(friction_smoothing_velocity_) || friction_smoothing_velocity_ <= 0.0 ||
      !std::isfinite(friction_validated_velocity_min_) || friction_validated_velocity_min_ < 0.0) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid friction calibration scalar: require scale >= 0, smoothing > 0, and v_min >= 0");
    return CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(friction_static_[i]) || friction_static_[i] < 0.0 ||
        (friction_stribeck_enabled_ && friction_static_[i] + 1.0e-8 < friction_coulomb_[i]) ||
        !std::isfinite(friction_stribeck_velocity_[i]) || friction_stribeck_velocity_[i] <= 0.0 ||
        !std::isfinite(friction_max_compensation_torque_[i]) ||
        friction_max_compensation_torque_[i] <= 0.0 ||
        !std::isfinite(friction_validated_velocity_min_per_joint_[i]) ||
        friction_validated_velocity_min_per_joint_[i] < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid Stribeck friction or compensation limit for joint %zu", i + 1);
      return CallbackReturn::ERROR;
    }
    if (!std::isfinite(friction_coulomb_[i]) || friction_coulomb_[i] < 0.0 ||
        !std::isfinite(friction_viscous_[i]) || friction_viscous_[i] < 0.0 ||
        !std::isfinite(friction_validated_velocity_max_[i]) ||
        friction_validated_velocity_max_[i] <= friction_validated_velocity_min_per_joint_[i]) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid friction coefficient/range for joint %zu", i + 1);
      return CallbackReturn::ERROR;
    }
  }

  friction_compensation_active_ =
      friction_compensation_requested_ && friction_calibration_valid_ &&
      friction_compensation_scale_ > 0.0 &&
      std::any_of(friction_joint_enable_.begin(), friction_joint_enable_.end(),
                  [](bool enabled) { return enabled; });

  use_mass_damping_ = std::any_of(
      mass_damping_.begin(), mass_damping_.end(),
      [](double gain) { return std::abs(gain) > 0.0; });

  if (use_mass_damping_) {
    const std::string interface_prefix =
        (arm_prefix_.empty() ? std::string{} : arm_prefix_ + "_") + robot_type_ + "/";
    franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
        interface_prefix + kRobotModelInterfaceName,
        interface_prefix + kRobotStateInterfaceName);
  } else {
    franka_robot_model_.reset();
  }

  ImpedanceCommand initial;
  command_buffer_.writeFromNonRT(initial);
  last_tau_command_.fill(0.0);
  timeout_hold_initialized_ = false;

  command_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      command_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
      std::bind(&JointImpedanceController::command_callback, this, std::placeholders::_1));
  ready_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/ready", rclcpp::QoS(1).transient_local().reliable());

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured impedance controller on topic '%s' (best-effort latest-value commands)",
              command_topic_.c_str());

  if (friction_compensation_active_) {
    std::string mask;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      mask += friction_joint_enable_[i] ? "1" : "0";
      if (i + 1 < kNumJoints) mask += ",";
    }
    RCLCPP_INFO(
        get_node()->get_logger(),
        "Friction compensation ACTIVE: source='%s', age=%.2f h, scale=%.3f, "
        "enabled=[%s], epsilon=%.4f rad/s; "
        "all-speed strictly odd %s model, no feedforward on command timeout, torque bias excluded",
        friction_calibration_source_.c_str(), friction_calibration_age_hours_,
        friction_compensation_scale_, mask.c_str(), friction_smoothing_velocity_,
        friction_stribeck_enabled_ ? "Stribeck" : "Coulomb-viscous");
  } else if (friction_compensation_requested_) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Friction compensation requested but inactive: no fresh/valid calibration was supplied");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Friction compensation disabled by configuration");
  }
  return CallbackReturn::SUCCESS;
}

JointImpedanceController::CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State&)
{
  last_tau_command_.fill(0.0);
  timeout_hold_initialized_ = false;

  if (use_mass_damping_ && franka_robot_model_) {
    franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  }

  for (auto& interface : command_interfaces_) {
    if (!interface.set_value(0.0)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to initialize effort command interface");
      return CallbackReturn::ERROR;
    }
  }

  if (ready_pub_) {
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);
  }
  RCLCPP_INFO(get_node()->get_logger(), "Impedance controller is operational");
  return CallbackReturn::SUCCESS;
}

JointImpedanceController::CallbackReturn JointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State&)
{
  if (ready_pub_) {
    std_msgs::msg::Bool ready;
    ready.data = false;
    ready_pub_->publish(ready);
  }
  bool success = true;

  for (auto& interface : command_interfaces_) {
    if (!interface.set_value(0.0)) {
      success = false;
    }
  }

  if (use_mass_damping_ && franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }

  if (!success) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to zero one or more effort command interfaces");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration&) {
  (void)time;
  std::array<double, kNumJoints> q{};
  std::array<double, kNumJoints> dq{};
  if (!read_joint_state(q, dq)) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to read joint state interfaces");
    return controller_interface::return_type::ERROR;
  }

  const auto* command = command_buffer_.readFromRT();
  const auto steady_now = std::chrono::steady_clock::now();
  const bool timed_out =
      command == nullptr || !command->valid ||
      std::chrono::duration<double>(steady_now - command->received_at).count() > command_timeout_s_;

  if (!timed_out) {
    timeout_hold_initialized_ = false;
  } else if (hold_position_on_timeout_ && !timeout_hold_initialized_) {
    timeout_hold_position_ = q;
    timeout_hold_initialized_ = true;
  }

  std::array<double, kNumJoints> tau_desired{};
  std::array<double, kNumJoints> velocity_error{};
  bool apply_velocity_feedback = false;

  if (!timed_out) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (command->has_position) {
        tau_desired[i] += stiffness_[i] * (command->position[i] - q[i]);
      }
      if (command->has_velocity) {
        velocity_error[i] = command->velocity[i] - dq[i];
        apply_velocity_feedback = true;
      } else if (command->has_position) {
        velocity_error[i] = -dq[i];
        apply_velocity_feedback = true;
      }
      if (command->has_effort) {
        tau_desired[i] += effort_feedforward_scale_ * command->effort[i];
      }
    }
  } else if (hold_position_on_timeout_) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      tau_desired[i] = stiffness_[i] * (timeout_hold_position_[i] - q[i]);
      velocity_error[i] = -dq[i];
    }
    apply_velocity_feedback = true;
  }

  if (apply_velocity_feedback) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      tau_desired[i] += damping_[i] * velocity_error[i];
    }

    if (use_mass_damping_) {
      if (!franka_robot_model_) {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "mass_damping is enabled but FrankaRobotModel is not configured");
        return controller_interface::return_type::ERROR;
      }

      const std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
      std::array<double, kNumJoints> scaled_velocity_error{};
      for (std::size_t j = 0; j < kNumJoints; ++j) {
        scaled_velocity_error[j] = mass_damping_[j] * velocity_error[j];
      }

      // libfranka stores the 7x7 mass matrix in column-major order.
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        double mass_term = 0.0;
        for (std::size_t j = 0; j < kNumJoints; ++j) {
          mass_term += mass[i + kNumJoints * j] * scaled_velocity_error[j];
        }
        tau_desired[i] += mass_term;
      }
    }
  }

  // Friction feedforward is only permitted while a fresh command is present.
  // On timeout, do not inject an unmatched active torque. Position-hold (if
  // configured) and the existing torque/rate limits still run normally.
  if (!timed_out && friction_compensation_active_) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (friction_joint_enable_[i]) {
        tau_desired[i] += friction_compensation_torque(i, dq[i]);
      }
    }
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    double tau = tau_desired[i];

    if (max_torque_[i] > 0.0) {
      tau = std::clamp(tau, -max_torque_[i], max_torque_[i]);
    }

    if (delta_tau_max_ > 0.0) {
      const double delta = std::clamp(
          tau - last_tau_command_[i], -delta_tau_max_, delta_tau_max_);
      tau = last_tau_command_[i] + delta;
    }

    if (!command_interfaces_[i].set_value(tau)) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Failed to write effort command for joint %zu", i);
      return controller_interface::return_type::ERROR;
    }
    last_tau_command_[i] = tau;
  }

  return controller_interface::return_type::OK;
}

void JointImpedanceController::command_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  ImpedanceCommand command;
  command.has_position = extract_field(*msg, msg->position, command.position);
  command.has_velocity = extract_field(*msg, msg->velocity, command.velocity);
  command.has_effort = extract_field(*msg, msg->effort, command.effort);

  if (!command.has_position && !command.has_velocity && !command.has_effort) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Ignoring JointState command: no complete 7-joint position, velocity, or effort field");
    return;
  }

  command.received_at = std::chrono::steady_clock::now();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

bool JointImpedanceController::extract_field(
    const sensor_msgs::msg::JointState& msg,
    const std::vector<double>& field,
    std::array<double, kNumJoints>& output) const {
  if (field.empty()) {
    return false;
  }
  if (msg.name.empty()) {
    if (field.size() != kNumJoints) {
      return false;
    }
    std::copy(field.begin(), field.end(), output.begin());
    return true;
  }
  if (field.size() != msg.name.size()) {
    return false;
  }

  for (std::size_t joint_i = 0; joint_i < kNumJoints; ++joint_i) {
    bool found = false;
    for (std::size_t msg_i = 0; msg_i < msg.name.size(); ++msg_i) {
      if (msg.name[msg_i] == joint_names_[joint_i]) {
        output[joint_i] = field[msg_i];
        found = true;
        break;
      }
    }
    if (!found) return false;
  }
  return true;
}

std::vector<std::string> JointImpedanceController::derive_joint_names() const {
  std::vector<std::string> names;
  names.reserve(kNumJoints);
  const std::string prefix = (arm_prefix_.empty() ? std::string{} : arm_prefix_ + "_") + robot_type_;
  for (std::size_t i = 1; i <= kNumJoints; ++i) {
    names.push_back(prefix + "_joint" + std::to_string(i));
  }
  return names;
}

double JointImpedanceController::friction_compensation_torque(
    std::size_t joint_i, double dq) const {
  // Use the measured velocity directly at every speed (no velocity clipping
  // or low-speed taper). The calibration's velocity band remains a diagnostic
  // validity range, not a runtime cutoff. Torque is always bounded below.
  double fc_effective = friction_coulomb_[joint_i];
  if (friction_stribeck_enabled_) {
    const double x = dq / friction_stribeck_velocity_[joint_i];
    fc_effective += (friction_static_[joint_i] - fc_effective) * std::exp(-x * x);
  }
  // Strictly odd, so friction feedforward remains zero at exactly zero speed.
  // The static-friction amplitude is not a commanded breakaway torque.
  const double tau = fc_effective * std::tanh(dq / friction_smoothing_velocity_) +
                     friction_viscous_[joint_i] * dq;
  const double limit = friction_max_compensation_torque_[joint_i];
  return std::clamp(friction_compensation_scale_ * tau, -limit, limit);
}

bool JointImpedanceController::read_joint_state(
    std::array<double, kNumJoints>& q,
    std::array<double, kNumJoints>& dq) const
{
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto q_value = state_interfaces_[2 * i].get_optional();
    const auto dq_value = state_interfaces_[2 * i + 1].get_optional();

    if (!q_value.has_value() || !dq_value.has_value()) {
      return false;
    }

    q[i] = q_value.value();
    dq[i] = dq_value.value();
  }

  return true;
}

}  // namespace low_level

PLUGINLIB_EXPORT_CLASS(low_level::JointImpedanceController, controller_interface::ControllerInterface)
