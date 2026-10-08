#include "low_level/dynamics_observer_controller.hpp"

#include <algorithm>
#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace low_level {

controller_interface::InterfaceConfiguration
DynamicsObserverController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
DynamicsObserverController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  if (franka_robot_model_) {
    const auto model_interfaces = franka_robot_model_->get_state_interface_names();
    config.names.insert(config.names.end(), model_interfaces.begin(), model_interfaces.end());
  }
  return config;
}

DynamicsObserverController::CallbackReturn DynamicsObserverController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::vector<std::string>>("joint_names", {});
    auto_declare<std::string>("observation_topic", "/fr3/dynamics_observation");
    auto_declare<double>("publish_rate", 200.0);
    auto_declare<double>("acceleration_filter_tau", 0.03);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

DynamicsObserverController::CallbackReturn DynamicsObserverController::on_configure(
    const rclcpp_lifecycle::State&) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  observation_topic_ = get_node()->get_parameter("observation_topic").as_string();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  acceleration_filter_tau_ = get_node()->get_parameter("acceleration_filter_tau").as_double();

  if (joint_names_.empty()) {
    joint_names_ = derive_joint_names();
  }
  if (joint_names_.size() != kNumJoints) {
    RCLCPP_ERROR(get_node()->get_logger(), "joint_names must contain exactly 7 names");
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0 || publish_rate_ > 1000.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "publish_rate must be in (0, 1000] Hz");
    return CallbackReturn::ERROR;
  }
  if (!std::isfinite(acceleration_filter_tau_) || acceleration_filter_tau_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "acceleration_filter_tau must be > 0");
    return CallbackReturn::ERROR;
  }

  const std::string interface_prefix =
      (arm_prefix_.empty() ? std::string{} : arm_prefix_ + "_") + robot_type_ + "/";
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      interface_prefix + kRobotModelInterfaceName,
      interface_prefix + kRobotStateInterfaceName);

  publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      observation_topic_, rclcpp::QoS(4).best_effort().durability_volatile());
  ready_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/ready", rclcpp::QoS(1).transient_local().reliable());
  realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
          publisher_);
  realtime_publisher_->msg_.data.resize(kObservationSize, 0.0);

  have_previous_velocity_ = false;
  previous_velocity_.fill(0.0);
  filtered_acceleration_.fill(0.0);
  publish_accumulator_ = 0.0;

  RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured dynamics observer on '%s' at %.1f Hz. Payload: q,dq,ddq,tau_meas,"
      "tau_Mddq,c,g,tau_model,residual,M (112 doubles).",
      observation_topic_.c_str(), publish_rate_);
  return CallbackReturn::SUCCESS;
}

DynamicsObserverController::CallbackReturn DynamicsObserverController::on_activate(
    const rclcpp_lifecycle::State&) {
  if (!franka_robot_model_) {
    return CallbackReturn::ERROR;
  }
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  have_previous_velocity_ = false;
  previous_velocity_.fill(0.0);
  filtered_acceleration_.fill(0.0);
  publish_accumulator_ = 0.0;
  if (ready_pub_) {
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);
  }
  RCLCPP_INFO(get_node()->get_logger(), "Dynamics observer is operational");
  return CallbackReturn::SUCCESS;
}

DynamicsObserverController::CallbackReturn DynamicsObserverController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  if (ready_pub_) {
    std_msgs::msg::Bool ready;
    ready.data = false;
    ready_pub_->publish(ready);
  }
  if (franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type DynamicsObserverController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  std::array<double, kNumJoints> q{};
  std::array<double, kNumJoints> dq{};
  std::array<double, kNumJoints> tau_measured{};
  if (!read_joint_state(q, dq, tau_measured)) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to read joint state interfaces in dynamics observer");
    return controller_interface::return_type::ERROR;
  }

  if (!have_previous_velocity_) {
    previous_velocity_ = dq;
    have_previous_velocity_ = true;
  } else {
    const double alpha = dt / (acceleration_filter_tau_ + dt);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const double raw_ddq = (dq[i] - previous_velocity_[i]) / dt;
      filtered_acceleration_[i] += alpha * (raw_ddq - filtered_acceleration_[i]);
      previous_velocity_[i] = dq[i];
    }
  }

  publish_accumulator_ += dt;
  const double publish_period = 1.0 / publish_rate_;
  if (publish_accumulator_ < publish_period) {
    return controller_interface::return_type::OK;
  }
  publish_accumulator_ = std::fmod(publish_accumulator_, publish_period);

  if (!realtime_publisher_ || !realtime_publisher_->trylock()) {
    return controller_interface::return_type::OK;
  }

  const std::array<double, 49> mass = franka_robot_model_->getMassMatrix();
  const std::array<double, 7> coriolis = franka_robot_model_->getCoriolisForceVector();
  const std::array<double, 7> gravity = franka_robot_model_->getGravityForceVector();

  std::array<double, kNumJoints> tau_inertia{};
  std::array<double, kNumJoints> tau_model{};
  std::array<double, kNumJoints> residual{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    double value = 0.0;
    for (std::size_t j = 0; j < kNumJoints; ++j) {
      value += mass[i + kNumJoints * j] * filtered_acceleration_[j];
    }
    tau_inertia[i] = value;
    tau_model[i] = tau_inertia[i] + coriolis[i] + gravity[i];
    residual[i] = tau_measured[i] - tau_model[i];
  }

  auto& data = realtime_publisher_->msg_.data;
  // 0..62: nine seven-element vectors.
  const std::array<const std::array<double, 7>*, 9> vectors = {
      &q, &dq, &filtered_acceleration_, &tau_measured, &tau_inertia,
      &coriolis, &gravity, &tau_model, &residual};
  std::size_t offset = 0;
  for (const auto* vector : vectors) {
    for (double value : *vector) {
      data[offset++] = value;
    }
  }
  // 63..111: Franka 7x7 mass matrix, column-major as returned by libfranka.
  for (double value : mass) {
    data[offset++] = value;
  }

  realtime_publisher_->unlockAndPublish();
  return controller_interface::return_type::OK;
}

std::vector<std::string> DynamicsObserverController::derive_joint_names() const {
  std::vector<std::string> names;
  names.reserve(kNumJoints);
  const std::string prefix =
      (arm_prefix_.empty() ? std::string{} : arm_prefix_ + "_") + robot_type_;
  for (std::size_t i = 1; i <= kNumJoints; ++i) {
    names.push_back(prefix + "_joint" + std::to_string(i));
  }
  return names;
}

bool DynamicsObserverController::read_joint_state(
    std::array<double, kNumJoints>& q,
    std::array<double, kNumJoints>& dq,
    std::array<double, kNumJoints>& tau) const {
  // The first 21 requested state interfaces are q,dq,effort in joint order.
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto q_value = state_interfaces_[3 * i].get_optional();
    const auto dq_value = state_interfaces_[3 * i + 1].get_optional();
    const auto tau_value = state_interfaces_[3 * i + 2].get_optional();
    if (!q_value.has_value() || !dq_value.has_value() || !tau_value.has_value()) {
      return false;
    }
    q[i] = q_value.value();
    dq[i] = dq_value.value();
    tau[i] = tau_value.value();
  }
  return true;
}

}  // namespace low_level

PLUGINLIB_EXPORT_CLASS(low_level::DynamicsObserverController,
                       controller_interface::ControllerInterface)
