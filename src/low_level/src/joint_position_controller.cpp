#include "low_level/joint_position_controller.hpp"
#include <algorithm>
#include <cmath>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace low_level {
namespace {
template<std::size_t N>
bool parse_limits(const std::vector<double>& source, std::array<double,N>& destination,
                  bool allow_zero) {
  if (source.size() != N) return false;
  for (std::size_t i = 0; i < N; ++i) {
    if (!std::isfinite(source[i]) || source[i] < 0.0 || (!allow_zero && source[i] == 0.0))
      return false;
    destination[i] = source[i];
  }
  return true;
}
}

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration out;
  out.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : joint_names_) out.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  return out;
}
controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration out;
  out.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& name : joint_names_) {
    out.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    out.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return out;
}
JointPositionController::CallbackReturn JointPositionController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::vector<std::string>>("joint_names", {});
    auto_declare<std::string>("command_topic", "/fr3/joint_position_commands");
    auto_declare<double>("command_timeout", 0.1);
    auto_declare<double>("position_kp", 1.0);
    auto_declare<double>("tracking_time_constant", 0.03);
    auto_declare<std::vector<double>>("max_velocity", std::vector<double>(N, 0.0));
    auto_declare<std::vector<double>>("max_acceleration", std::vector<double>(N, 2.0));
    auto_declare<std::vector<double>>("max_jerk", std::vector<double>(N, 100.0));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
JointPositionController::CallbackReturn JointPositionController::on_configure(const rclcpp_lifecycle::State&) {
  const auto robot = get_node()->get_parameter("robot_type").as_string();
  const auto prefix = get_node()->get_parameter("arm_prefix").as_string();
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  if (joint_names_.empty()) {
    const std::string base = (prefix.empty() ? "" : prefix + "_") + robot;
    for (std::size_t i = 1; i <= N; ++i) joint_names_.push_back(base + "_joint" + std::to_string(i));
  }
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  timeout_ = get_node()->get_parameter("command_timeout").as_double();
  kp_ = get_node()->get_parameter("position_kp").as_double();
  tau_ = get_node()->get_parameter("tracking_time_constant").as_double();
  if (joint_names_.size() != N || !std::isfinite(timeout_) || timeout_ <= 0.0 ||
      !std::isfinite(kp_) || kp_ < 0.0 || !std::isfinite(tau_) || tau_ <= 0.0 ||
      !parse_limits(get_node()->get_parameter("max_velocity").as_double_array(), max_velocity_, true) ||
      !parse_limits(get_node()->get_parameter("max_acceleration").as_double_array(), max_acceleration_, false) ||
      !parse_limits(get_node()->get_parameter("max_jerk").as_double_array(), max_jerk_, false)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid joint names, gains, timeout, or motion shaping limits");
    return CallbackReturn::ERROR;
  }
  buffer_.writeFromNonRT(PositionCommand{});
  previous_velocity_.fill(0.0);
  previous_acceleration_.fill(0.0);
  subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      command_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
      std::bind(&JointPositionController::callback, this, std::placeholders::_1));
  ready_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/ready", rclcpp::QoS(1).transient_local().reliable());
  RCLCPP_INFO(get_node()->get_logger(), "Position P controller on %s, Kp=%.3f 1/s", command_topic_.c_str(), kp_);
  return CallbackReturn::SUCCESS;
}
JointPositionController::CallbackReturn JointPositionController::on_activate(const rclcpp_lifecycle::State&) {
  buffer_.writeFromNonRT(PositionCommand{}); // Require a NEW message after activation.
  previous_velocity_.fill(0.0);
  previous_acceleration_.fill(0.0);
  for (auto& iface : command_interfaces_) if (!iface.set_value(0.0)) return CallbackReturn::ERROR;
  std_msgs::msg::Bool ready; ready.data = true; ready_pub_->publish(ready);
  return CallbackReturn::SUCCESS;
}
JointPositionController::CallbackReturn JointPositionController::on_deactivate(const rclcpp_lifecycle::State&) {
  buffer_.writeFromNonRT(PositionCommand{});
  previous_velocity_.fill(0.0);
  previous_acceleration_.fill(0.0);
  for (auto& iface : command_interfaces_) if (!iface.set_value(0.0)) return CallbackReturn::ERROR;
  std_msgs::msg::Bool ready; ready.data = false; ready_pub_->publish(ready);
  return CallbackReturn::SUCCESS;
}
void JointPositionController::callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  PositionCommand value;
  if (msg->name.empty()) {
    if (msg->position.size() != N) return;
    std::copy(msg->position.begin(), msg->position.end(), value.position.begin());
  } else {
    if (msg->name.size() != msg->position.size()) return;
    for (std::size_t i = 0; i < N; ++i) {
      bool found = false;
      for (std::size_t j = 0; j < msg->name.size(); ++j) {
        if (msg->name[j] == joint_names_[i]) {
          value.position[i] = msg->position[j];
          found = true;
          break;
        }
      }
      if (!found) return;
    }
  }
  for (double q : value.position) if (!std::isfinite(q)) return;
  value.valid = true;
  value.received_at = std::chrono::steady_clock::now();
  buffer_.writeFromNonRT(value);
}
controller_interface::return_type JointPositionController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) return controller_interface::return_type::OK;
  const auto* command = buffer_.readFromRT();
  const bool valid = command && command->valid &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - command->received_at).count() <= timeout_;
  std::array<double,N> target{};
  if (valid) {
    for (std::size_t i = 0; i < N; ++i) {
      const double q = state_interfaces_[2*i].get_value();
      if (!std::isfinite(q)) return controller_interface::return_type::ERROR;
      target[i] = kp_ * (command->position[i] - q);
    }
    // Optional direction-preserving velocity limit (zero disables only this limit).
    double ratio = 1.0;
    for (std::size_t i = 0; i < N; ++i)
      if (max_velocity_[i] > 0.0) ratio = std::max(ratio, std::abs(target[i]) / max_velocity_[i]);
    for (double& v : target) v /= ratio;
  }
  // Shaping is identical to the existing velocity controller: no velocity discontinuities.
  std::array<double,N> desired_acc{}, acceleration_delta{}, acceleration{}, velocity{};
  double accel_ratio = 1.0;
  for (std::size_t i = 0; i < N; ++i) {
    desired_acc[i] = (target[i] - previous_velocity_[i]) / tau_;
    accel_ratio = std::max(accel_ratio, std::abs(desired_acc[i]) / max_acceleration_[i]);
  }
  for (double& a : desired_acc) a /= accel_ratio;
  double jerk_ratio = 1.0;
  for (std::size_t i = 0; i < N; ++i) {
    acceleration_delta[i] = desired_acc[i] - previous_acceleration_[i];
    jerk_ratio = std::max(jerk_ratio, std::abs(acceleration_delta[i]) / (max_jerk_[i] * dt));
  }
  for (double& delta : acceleration_delta) delta /= jerk_ratio;
  for (std::size_t i = 0; i < N; ++i) {
    acceleration[i] = previous_acceleration_[i] + acceleration_delta[i];
    velocity[i] = previous_velocity_[i] + acceleration[i] * dt;
    if (!std::isfinite(velocity[i]) || !command_interfaces_[i].set_value(velocity[i]))
      return controller_interface::return_type::ERROR;
  }
  previous_velocity_ = velocity;
  previous_acceleration_ = acceleration;
  return controller_interface::return_type::OK;
}
} // namespace low_level
PLUGINLIB_EXPORT_CLASS(low_level::JointPositionController, controller_interface::ControllerInterface)
