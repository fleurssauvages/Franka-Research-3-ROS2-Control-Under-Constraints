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
    if (gravity_error_compensation_enabled_ || breakaway_enabled_) {
      config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }
  }
  if ((use_mass_damping_ || gravity_error_compensation_enabled_ || breakaway_enabled_) && franka_robot_model_) {
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
    auto_declare<std::vector<double>>("damping", std::vector<double>(kNumJoints, 0.0));
    auto_declare<std::vector<double>>("mass_damping", std::vector<double>(kNumJoints, 0.0));
    auto_declare<double>("effort_feedforward_scale", 1.0);
    auto_declare<double>("delta_tau_max", 1.0);
    auto_declare<std::vector<double>>("max_torque", std::vector<double>(kNumJoints, 0.0));
    auto_declare<bool>("hold_position_on_timeout", false);

    auto_declare<bool>("gravity_error_compensation_enabled", false);
    auto_declare<double>("gravity_error_calibration_lease_s", 0.35);
    auto_declare<double>("gravity_error_command_velocity_epsilon", 0.002);
    auto_declare<double>("gravity_error_measured_velocity_epsilon", 0.003);
    auto_declare<double>("gravity_error_stationary_dwell_s", 0.5);
    auto_declare<double>("gravity_error_minimum_sample_s", 1.0);
    auto_declare<double>("gravity_error_filter_tau_s", 2.0);
    auto_declare<double>("gravity_error_sampling_command_torque_epsilon", 0.03);
    auto_declare<double>("gravity_error_sample_deviation_limit", 0.10);
    auto_declare<double>("gravity_error_output_slew_rate", 0.10);
    auto_declare<double>("gravity_error_pose_radius", 0.25);
    auto_declare<double>("gravity_error_pose_fade_width", 0.25);
    auto_declare<std::vector<double>>(
        "gravity_error_max_torque", std::vector<double>(kNumJoints, 0.15));

    auto_declare<bool>("breakaway_enabled", false);
    auto_declare<double>("breakaway_velocity_epsilon", 0.004);
    auto_declare<double>("breakaway_external_torque_deadband", 0.6);
    auto_declare<double>("breakaway_gain", 0.10);
    auto_declare<double>("breakaway_max_torque", 0.08);
    auto_declare<double>("breakaway_slew_rate", 0.10);
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
  gravity_error_compensation_enabled_ =
      get_node()->get_parameter("gravity_error_compensation_enabled").as_bool();
  gravity_error_calibration_lease_s_ =
      get_node()->get_parameter("gravity_error_calibration_lease_s").as_double();
  residual_settings_.command_velocity_epsilon =
      get_node()->get_parameter("gravity_error_command_velocity_epsilon").as_double();
  residual_settings_.measured_velocity_epsilon =
      get_node()->get_parameter("gravity_error_measured_velocity_epsilon").as_double();
  residual_settings_.stationary_dwell_s =
      get_node()->get_parameter("gravity_error_stationary_dwell_s").as_double();
  residual_settings_.minimum_sample_s =
      get_node()->get_parameter("gravity_error_minimum_sample_s").as_double();
  residual_settings_.filter_tau_s =
      get_node()->get_parameter("gravity_error_filter_tau_s").as_double();
  residual_settings_.sampling_command_torque_epsilon =
      get_node()->get_parameter("gravity_error_sampling_command_torque_epsilon").as_double();
  residual_settings_.sample_deviation_limit =
      get_node()->get_parameter("gravity_error_sample_deviation_limit").as_double();
  residual_settings_.output_slew_rate =
      get_node()->get_parameter("gravity_error_output_slew_rate").as_double();
  residual_settings_.pose_radius =
      get_node()->get_parameter("gravity_error_pose_radius").as_double();
  residual_settings_.pose_fade_width =
      get_node()->get_parameter("gravity_error_pose_fade_width").as_double();

  breakaway_enabled_ = get_node()->get_parameter("breakaway_enabled").as_bool();
  breakaway_velocity_epsilon_ = get_node()->get_parameter("breakaway_velocity_epsilon").as_double();
  breakaway_deadband_ = get_node()->get_parameter("breakaway_external_torque_deadband").as_double();
  breakaway_gain_ = get_node()->get_parameter("breakaway_gain").as_double();
  breakaway_max_torque_ = get_node()->get_parameter("breakaway_max_torque").as_double();
  breakaway_slew_rate_ = get_node()->get_parameter("breakaway_slew_rate").as_double();
  if (!std::isfinite(breakaway_velocity_epsilon_) || breakaway_velocity_epsilon_ <= 0.0 ||
      !std::isfinite(breakaway_deadband_) || breakaway_deadband_ < 0.0 ||
      !std::isfinite(breakaway_gain_) || breakaway_gain_ < 0.0 ||
      !std::isfinite(breakaway_max_torque_) || breakaway_max_torque_ <= 0.0 ||
      !std::isfinite(breakaway_slew_rate_) || breakaway_slew_rate_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid breakaway configuration");
    return CallbackReturn::ERROR;
  }
  breakaway_applied_.fill(0.0);
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

  if (gravity_error_compensation_enabled_) {
    const bool valid_settings =
        std::isfinite(gravity_error_calibration_lease_s_) && gravity_error_calibration_lease_s_ > 0.0 &&
        std::isfinite(residual_settings_.command_velocity_epsilon) &&
        residual_settings_.command_velocity_epsilon > 0.0 &&
        std::isfinite(residual_settings_.measured_velocity_epsilon) &&
        residual_settings_.measured_velocity_epsilon > 0.0 &&
        std::isfinite(residual_settings_.stationary_dwell_s) &&
        residual_settings_.stationary_dwell_s > 0.0 &&
        std::isfinite(residual_settings_.minimum_sample_s) &&
        residual_settings_.minimum_sample_s > 0.0 &&
        std::isfinite(residual_settings_.filter_tau_s) &&
        residual_settings_.filter_tau_s > 0.0 &&
        std::isfinite(residual_settings_.sampling_command_torque_epsilon) &&
        residual_settings_.sampling_command_torque_epsilon >= 0.0 &&
        std::isfinite(residual_settings_.sample_deviation_limit) &&
        residual_settings_.sample_deviation_limit > 0.0 &&
        std::isfinite(residual_settings_.output_slew_rate) &&
        residual_settings_.output_slew_rate > 0.0 &&
        std::isfinite(residual_settings_.pose_radius) &&
        residual_settings_.pose_radius >= 0.0 &&
        std::isfinite(residual_settings_.pose_fade_width) &&
        residual_settings_.pose_fade_width > 0.0 &&
        copy_parameter_array(get_node()->get_parameter("gravity_error_max_torque").as_double_array(),
                             residual_settings_.torque_limits);
    if (!valid_settings ||
        std::any_of(residual_settings_.torque_limits.begin(), residual_settings_.torque_limits.end(),
                    [](double limit) { return !std::isfinite(limit) || limit <= 0.0; })) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid residual compensation configuration");
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

  if (use_mass_damping_ || gravity_error_compensation_enabled_ || breakaway_enabled_) {
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
  breakaway_applied_.fill(0.0);
  timeout_hold_initialized_ = false;
  residual_compensator_.reset();
  residual_realtime_pub_.reset();
  residual_pub_.reset();
  calibration_sub_.reset();
  if (gravity_error_compensation_enabled_) {
    residual_compensator_ = std::make_unique<ResidualTorqueCompensator>(residual_settings_);
    CalibrationAuthorization disabled;
    calibration_buffer_.writeFromNonRT(disabled);
    calibration_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        "~/gravity_error_calibration_enable", rclcpp::QoS(1).reliable().durability_volatile(),
        std::bind(&JointImpedanceController::calibration_callback, this, std::placeholders::_1));
    residual_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/gravity_error_state", rclcpp::QoS(2).best_effort().durability_volatile());
    residual_realtime_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
            residual_pub_);
    // diagnostic: 0..6 estimate, 7..13 applied, 14 calibrated, 15 collecting
    residual_realtime_pub_->msg_.data.resize(16, 0.0);
    residual_publish_s_ = 0.0;
    RCLCPP_WARN(get_node()->get_logger(),
        "EXPERIMENTAL residual calibration enabled. No contact is not detectable "
        "from zero velocity: authorize only while the arm is externally unloaded. "
        "Repeated Bool true messages on ~/gravity_error_calibration_enable required.");
  }

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
        "all-speed strictly odd %s model, friction remains active on command timeout",
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
  breakaway_applied_.fill(0.0);
  timeout_hold_initialized_ = false;
  if (residual_compensator_) residual_compensator_->reset();
  residual_publish_s_ = 0.0;
  CalibrationAuthorization disabled;
  calibration_buffer_.writeFromNonRT(disabled);

  if ((use_mass_damping_ || gravity_error_compensation_enabled_ || breakaway_enabled_) && franka_robot_model_) {
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

  if ((use_mass_damping_ || gravity_error_compensation_enabled_ || breakaway_enabled_) && franka_robot_model_) {
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
    const rclcpp::Time& time, const rclcpp::Duration& period) {
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

  // Manual-guidance friction compensation is independent of command freshness.
  // Command timeout continues to gate position/velocity/effort feedforward.
  if (friction_compensation_active_) {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (friction_joint_enable_[i]) {
        tau_desired[i] += friction_compensation_torque(i, dq[i]);
      }
    }
  }

  // Experimental breakaway assist (OFF by default). This uses a torque-residual
  // proxy; it is NOT a validated external wrench measurement. Positive feedback
  // must only be used after observing its behavior with the robot secured.
  if (breakaway_enabled_ && franka_robot_model_) {
    const auto gravity = franka_robot_model_->getGravityForceVector();
    const double dt = period.seconds();
    if (!std::isfinite(dt) || dt <= 0.0) return controller_interface::return_type::ERROR;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const auto measured = state_interfaces_[3 * i + 2].get_optional();
      if (!measured.has_value() || !std::isfinite(measured.value()) ||
          !std::isfinite(gravity[i])) return controller_interface::return_type::ERROR;
      // Previous control effort is subtracted to avoid positive feedback from our
      // own commanded torque, but internal actuator dynamics still contaminate it.
      const double external_proxy = measured.value() - gravity[i] - last_tau_command_[i];
      double target = 0.0;
      if (std::abs(dq[i]) <= breakaway_velocity_epsilon_) {
        const double magnitude = std::max(0.0, std::abs(external_proxy) - breakaway_deadband_);
        target = std::copysign(std::min(breakaway_max_torque_, breakaway_gain_ * magnitude),
                               external_proxy);
      }
      const double max_step = breakaway_slew_rate_ * dt;
      breakaway_applied_[i] += std::clamp(target - breakaway_applied_[i], -max_step, max_step);
      tau_desired[i] += breakaway_applied_[i];
    }
  }

  // Residual is sampled ONLY under an operator-authorized, unloaded calibration
  // lease, after all commanded torques have become approximately zero. It is
  // held (not adapted) while the operator moves the arm. Calibration torque is
  // switched off smoothly; correction is bounded independently of friction.
  if (residual_compensator_ && franka_robot_model_) {
    const std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();
    std::array<double, kNumJoints> tau_measured{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const auto torque = state_interfaces_[3 * i + 2].get_optional();
      if (!torque.has_value() || !std::isfinite(torque.value()) ||
          !std::isfinite(gravity[i])) {
        return controller_interface::return_type::ERROR;
      }
      tau_measured[i] = torque.value();
    }

    const auto* authorization = calibration_buffer_.readFromRT();
    const bool authorized = authorization != nullptr && authorization->enabled &&
        std::chrono::duration<double>(steady_now - authorization->received_at).count() <=
            gravity_error_calibration_lease_s_;
    // No sampling if another controller command is delivering nonzero torque.
    // The actual torque guard inside the estimator provides a second check.
    bool effort_command_zero = !timed_out;
    if (command != nullptr && command->has_effort) {
      for (double value : command->effort) {
        effort_command_zero = effort_command_zero &&
            std::abs(effort_feedforward_scale_ * value) <
                residual_settings_.sampling_command_torque_epsilon;
      }
    }
    const std::array<double, kNumJoints> commanded_velocity =
        (!timed_out && command->has_velocity) ? command->velocity
                                             : std::array<double, kNumJoints>{};
    const auto& correction = residual_compensator_->update(
        period.seconds(), !timed_out, !timed_out && command->has_velocity &&
        effort_command_zero, commanded_velocity, q, dq, tau_measured, gravity,
        last_tau_command_, authorized && effort_command_zero);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      tau_desired[i] += correction[i];
    }

    // Nonblocking, allocation-free periodic diagnostics from the RT loop.
    if (std::isfinite(period.seconds()) && period.seconds() > 0.0) {
      residual_publish_s_ += period.seconds();
    }
    if (residual_publish_s_ >= 0.1 && residual_realtime_pub_ &&
        residual_realtime_pub_->trylock()) {
      residual_publish_s_ = 0.0;
      auto& data = residual_realtime_pub_->msg_.data;
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        data[i] = residual_compensator_->estimate()[i];
        data[7 + i] = residual_compensator_->applied()[i];
      }
      data[14] = residual_compensator_->calibrated() ? 1.0 : 0.0;
      data[15] = residual_compensator_->collecting() ? 1.0 : 0.0;
      residual_realtime_pub_->unlockAndPublish();
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

void JointImpedanceController::calibration_callback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  CalibrationAuthorization authorization;
  authorization.enabled = msg->data;
  authorization.received_at = std::chrono::steady_clock::now();
  calibration_buffer_.writeFromNonRT(authorization);
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
    if (!std::all_of(field.begin(), field.end(),
                     [](double value) { return std::isfinite(value); })) return false;
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
        if (!std::isfinite(field[msg_i])) return false;
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
    const auto q_value = state_interfaces_[((gravity_error_compensation_enabled_ || breakaway_enabled_) ? 3 : 2) * i].get_optional();
    const auto dq_value = state_interfaces_[((gravity_error_compensation_enabled_ || breakaway_enabled_) ? 3 : 2) * i + 1].get_optional();

    if (!q_value.has_value() || !dq_value.has_value() ||
        !std::isfinite(q_value.value()) || !std::isfinite(dq_value.value())) {
      return false;
    }

    q[i] = q_value.value();
    dq[i] = dq_value.value();
  }

  return true;
}

}  // namespace low_level

PLUGINLIB_EXPORT_CLASS(low_level::JointImpedanceController, controller_interface::ControllerInterface)
