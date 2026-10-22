#include "franka_cartesian_control/ik_force_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace franka_cartesian_control {
namespace {

Vector6d vec6(const std::vector<double>& input, const char* name) {
  if (input.size() != 6) {
    throw std::runtime_error(std::string(name) + " must contain 6 values");
  }
  Vector6d result;
  for (int i = 0; i < 6; ++i) result(i) = input[i];
  if (!result.allFinite()) {
    throw std::runtime_error(std::string(name) + " must be finite");
  }
  return result;
}

Vector7d vec7(const std::vector<double>& input, const char* name) {
  if (input.size() != 7) {
    throw std::runtime_error(std::string(name) + " must contain 7 values");
  }
  Vector7d result;
  for (int i = 0; i < 7; ++i) result(i) = input[i];
  if (!result.allFinite()) {
    throw std::runtime_error(std::string(name) + " must be finite");
  }
  return result;
}

Vector6d wrenchToVector(const geometry_msgs::msg::WrenchStamped& msg) {
  Vector6d result;
  result << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
  return result;
}

}  // namespace

IkForceNode::IkForceNode() : Node("ik_force_controller") {
  const auto robot_type = declare_parameter<std::string>("robot_type", "fr3");
  const auto arm_prefix = declare_parameter<std::string>("arm_prefix", "");
  root_link_ = declare_parameter<std::string>("root_link", "");
  joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", deriveJointNames(robot_type, arm_prefix));
  if (joint_names_.size() != kDof) {
    throw std::runtime_error("joint_names must contain exactly 7 joint names");
  }
  if (root_link_.empty()) root_link_ = deriveRootLink(robot_type, arm_prefix);

  const auto input_mode = declare_parameter<std::string>("force_input_mode", "wrench");
  if (input_mode != "wrench" && input_mode != "twist") {
    throw std::runtime_error("force_input_mode must be 'wrench' or 'twist'");
  }
  twist_mode_ = input_mode == "twist";
  frequency_ = declare_parameter<double>("frequency", 200.0);
  command_timeout_ = declare_parameter<double>("command_timeout", 0.1);
  joint_state_timeout_ = declare_parameter<double>("joint_state_timeout", 0.2);
  pinv_damping_ = declare_parameter<double>("pinv_damping", 0.02);
  joint_limit_margin_ = declare_parameter<double>("joint_limit_margin", 0.08);
  joint_limit_gain_ = declare_parameter<double>("joint_limit_gain", 2.0);
  max_force_ = declare_parameter<double>("max_cartesian_force", 15.0);
  max_moment_ = declare_parameter<double>("max_cartesian_moment", 2.0);
  velocity_to_wrench_gains_ = vec6(declare_parameter<std::vector<double>>(
      "velocity_to_wrench_gains", {40., 40., 40., 5., 5., 5.}),
      "velocity_to_wrench_gains");
  max_velocity_ = vec7(declare_parameter<std::vector<double>>(
      "max_velocity", {1., 1., 1., 1., 1.5, 1.5, 1.5}), "max_velocity");
  max_joint_torque_ = vec7(declare_parameter<std::vector<double>>(
      "max_joint_torque", {8., 8., 8., 6., 4., 4., 4.}), "max_joint_torque");
  q_min_ = vec7(declare_parameter<std::vector<double>>(
      "joint_min", {-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508}),
      "joint_min");
  q_max_ = vec7(declare_parameter<std::vector<double>>(
      "joint_max", {2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508}),
      "joint_max");

  if (!std::isfinite(frequency_) || frequency_ <= 0.0 ||
      !std::isfinite(command_timeout_) || command_timeout_ <= 0.0 ||
      !std::isfinite(joint_state_timeout_) || joint_state_timeout_ <= 0.0 ||
      !std::isfinite(pinv_damping_) || pinv_damping_ <= 0.0 ||
      !std::isfinite(joint_limit_margin_) || joint_limit_margin_ < 0.0 ||
      !std::isfinite(joint_limit_gain_) || joint_limit_gain_ <= 0.0 ||
      !std::isfinite(max_force_) || max_force_ <= 0.0 ||
      !std::isfinite(max_moment_) || max_moment_ <= 0.0 ||
      (velocity_to_wrench_gains_.array() < 0.0).any() ||
      (max_velocity_.array() <= 0.0).any() ||
      (max_joint_torque_.array() <= 0.0).any() ||
      (q_max_.array() - q_min_.array() <= 2.0 * joint_limit_margin_).any()) {
    throw std::runtime_error("Invalid IK force controller gains, limits, or timeouts");
  }

  const auto robot_description_topic = declare_parameter<std::string>(
      "robot_description_topic", "/robot_description");
  const auto franka_state_topic = declare_parameter<std::string>(
      "franka_robot_state_topic", "/franka_robot_state_broadcaster/robot_state");
  const auto franka_state_timeout = declare_parameter<double>("franka_state_timeout", 0.5);
  kinematics_.subscribeToFrankaState(*this, franka_state_topic, franka_state_timeout);
  RCLCPP_INFO(get_logger(), "TCP: link8 kinematics + live Desk F_T_EE from %s (timeout %.3f s)",
              franka_state_topic.c_str(), franka_state_timeout);
  const auto joint_state_topic = declare_parameter<std::string>(
      "joint_state_topic", "/franka/joint_states");
  const auto twist_topic = declare_parameter<std::string>(
      "twist_topic", "/fr3/cartesian_twist_command");
  const auto wrench_topic = declare_parameter<std::string>(
      "wrench_topic", "/fr3/cartesian_wrench_command");
  const auto torque_topic = declare_parameter<std::string>(
      "torque_command_topic", "/fr3/tau_command");

  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&IkForceNode::robotDescriptionCallback, this, std::placeholders::_1));
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      std::bind(&IkForceNode::jointStateCallback, this, std::placeholders::_1));
  const auto command_qos = rclcpp::QoS(1).best_effort().durability_volatile();
  if (twist_mode_) {
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        twist_topic, command_qos,
        std::bind(&IkForceNode::twistCallback, this, std::placeholders::_1));
  } else {
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        wrench_topic, command_qos,
        std::bind(&IkForceNode::wrenchCallback, this, std::placeholders::_1));
  }
  command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      torque_topic, command_qos);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "~/ready", rclcpp::QoS(1).transient_local().reliable());
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_),
                             std::bind(&IkForceNode::update, this));
  RCLCPP_INFO(get_logger(), "IK force (%s): %s -> %s, output: %s (JointState.effort)",
              input_mode.c_str(), root_link_.c_str(), "Desk EE (via link8)", torque_topic.c_str());
}

bool IkForceNode::frameIsRoot(const std::string& frame) const {
  // Empty frame means the configured root frame, matching existing twist IK.
  // Other frames require an explicit TF transform, not an implicit rotation.
  return frame.empty() || frame == root_link_;
}

void IkForceNode::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (kinematics_ready_) return;
  std::string error;
  if (!kinematics_.initializeFromRobotDescription(msg->data, root_link_, &error)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
    return;
  }
  kinematics_ready_ = true;
  RCLCPP_INFO(get_logger(), "IK force kinematics initialized: %s -> %s",
              root_link_.c_str(), kinematics_.flangeLink().c_str());
}

void IkForceNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  Vector7d q, dq;
  // Unlike the legacy velocity helper, force feedback must not silently
  // substitute zero when actual joint velocities are absent.
  if (msg->velocity.size() != msg->name.size() ||
      !extractJointState(*msg, joint_names_, q, dq)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  q_ = q;
  dq_ = dq;
  last_state_time_ = std::chrono::steady_clock::now();
  have_joint_state_ = true;
}

void IkForceNode::wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  if (!frameIsRoot(msg->header.frame_id)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Ignoring wrench in frame '%s'; expected root frame '%s'",
                         msg->header.frame_id.c_str(), root_link_.c_str());
    return;
  }
  const Vector6d wrench = wrenchToVector(*msg);
  if (!wrench.allFinite()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  desired_cartesian_ = wrench;
  last_command_time_ = std::chrono::steady_clock::now();
  have_command_ = true;
}

void IkForceNode::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  if (!frameIsRoot(msg->header.frame_id)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Ignoring twist in frame '%s'; expected root frame '%s'",
                         msg->header.frame_id.c_str(), root_link_.c_str());
    return;
  }
  const Vector6d twist = twistToVector(*msg);
  if (!twist.allFinite()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  desired_cartesian_ = twist;
  last_command_time_ = std::chrono::steady_clock::now();
  have_command_ = true;
}

void IkForceNode::update() {
  Vector7d q, dq;
  Vector6d target;
  bool have_state, have_command;
  std::chrono::steady_clock::time_point state_time, command_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q = q_; dq = dq_; target = desired_cartesian_;
    have_state = have_joint_state_; have_command = have_command_;
    state_time = last_state_time_; command_time = last_command_time_;
  }
  const auto steady_now = std::chrono::steady_clock::now();
  const bool state_fresh = have_state &&
      std::chrono::duration<double>(steady_now - state_time).count() <= joint_state_timeout_;
  if (kinematics_.tcpReady() && state_fresh && !ready_published_) {
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);
    ready_published_ = true;
  }
  if (!kinematics_.tcpReady() || !state_fresh || !have_command) {
    if (ready_published_ && (!kinematics_.tcpReady() || !state_fresh)) {
      std_msgs::msg::Bool ready;
      ready.data = false;
      ready_pub_->publish(ready);
      ready_published_ = false;
    }
    // Do not keep a torque lease alive without valid feedback and a command.
    return;
  }
  if (std::chrono::duration<double>(steady_now - command_time).count() > command_timeout_) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_time == last_command_time_) have_command_ = false;
    // The lower-level watchdog slews the effort toward zero on timeout.
    return;
  }

  Matrix67d J;
  Eigen::Isometry3d pose;
  if (!kinematics_.compute(q, J, pose) || !J.allFinite()) return;

  Vector6d wrench = target;
  if (twist_mode_) {
    // The same damped differential IK used by the velocity controller first
    // projects the requested TCP twist onto feasible joint velocities. Then
    // Cartesian velocity feedback computes a force/moment, not a joint dq.
    Vector7d qdot_des = dampedPseudoInverse(J, pinv_damping_) * target;
    Vector7d lower, upper;
    combineJointBounds(q, max_velocity_, q_min_, q_max_, joint_limit_margin_,
                       joint_limit_gain_, lower, upper);
    qdot_des = saturateVectorPreservingDirection(qdot_des, lower, upper);
    wrench = velocity_to_wrench_gains_.cwiseProduct(J * (qdot_des - dq));
  }
  if (!wrench.allFinite()) return;

  // Independently bound linear force [N] and spatial moment [Nm] magnitudes.
  const double force_norm = wrench.head<3>().norm();
  const double moment_norm = wrench.tail<3>().norm();
  if (force_norm > max_force_) wrench.head<3>() *= max_force_ / force_norm;
  if (moment_norm > max_moment_) wrench.tail<3>() *= max_moment_ / moment_norm;

  // Virtual work: wrench is expressed in the KDL chain's ROOT frame, at its
  // resolved TCP. The Jacobian transpose produces joint torques [Nm].
  Vector7d tau = J.transpose() * wrench;
  if (!tau.allFinite()) return;
  // Uniform scaling preserves the commanded Cartesian wrench direction.
  double scale = 1.0;
  for (int i = 0; i < 7; ++i) {
    if (std::abs(tau(i)) > max_joint_torque_(i)) {
      scale = std::min(scale, max_joint_torque_(i) / std::abs(tau(i)));
    }
  }
  tau *= scale;

  sensor_msgs::msg::JointState command;
  command.header.stamp = now();
  command.name = joint_names_;
  command.effort.resize(kDof);
  for (std::size_t i = 0; i < kDof; ++i) command.effort[i] = tau(static_cast<int>(i));
  // Deliberately leave command.position and command.velocity EMPTY.
  command_pub_->publish(command);
}

}  // namespace franka_cartesian_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<franka_cartesian_control::IkForceNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ik_force_controller"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
