#include "franka_cartesian_control/pid_position_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <stdexcept>

#include "franka_cartesian_control/position_control_common.hpp"

namespace franka_cartesian_control {

PidPositionNode::PidPositionNode() : Node("pid_position_controller") {
  robot_type_ = declare_parameter<std::string>("robot_type", "fr3");
  arm_prefix_ = declare_parameter<std::string>("arm_prefix", "");
  root_link_ = declare_parameter<std::string>("root_link", "");
  tip_link_ = declare_parameter<std::string>("tip_link", "");
  joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", deriveJointNames(robot_type_, arm_prefix_));
  if (joint_names_.size() != 7) throw std::runtime_error("joint_names must contain 7 entries");
  if (root_link_.empty()) root_link_ = deriveRootLink(robot_type_, arm_prefix_);
  tip_link_ = deriveTipLink(robot_type_, arm_prefix_, tip_link_);

  frequency_ = declare_parameter<double>("frequency", 50.0);
  command_timeout_ = declare_parameter<double>("command_timeout", 0.50);
  reset_integral_on_new_target_ = declare_parameter<bool>("reset_integral_on_new_target", true);

  kp_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "kp", {5.0, 5.0, 5.0, 5.0, 5.0, 5.0}), "kp");
  ki_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "ki", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), "ki");
  kd_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "kd", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), "kd");
  integral_limit_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "integral_limit", {0.20, 0.20, 0.20, 0.20, 0.20, 0.20}), "integral_limit");
  max_twist_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "max_twist", {0.50, 0.50, 0.50, 0.20, 0.20, 0.20}), "max_twist");

  const auto robot_description_topic =
      declare_parameter<std::string>("robot_description_topic", "/robot_description");
  const auto joint_state_topic =
      declare_parameter<std::string>("joint_state_topic", "/franka/joint_states");
  const auto desired_pose_topic =
      declare_parameter<std::string>("desired_pose_topic", "/fr3/cartesian_pose_command");
  const auto twist_topic =
      declare_parameter<std::string>("twist_topic", "/fr3/cartesian_twist_command");
  const auto current_pose_topic =
      declare_parameter<std::string>("current_pose_topic", "/fr3/cartesian_pose");

  auto robot_qos = rclcpp::QoS(1).transient_local().reliable();
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic, robot_qos,
      std::bind(&PidPositionNode::robotDescriptionCallback, this, std::placeholders::_1));
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      std::bind(&PidPositionNode::jointStateCallback, this, std::placeholders::_1));
  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      desired_pose_topic, rclcpp::QoS(1).reliable(),
      std::bind(&PidPositionNode::desiredPoseCallback, this, std::placeholders::_1));
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      twist_topic, rclcpp::QoS(1).best_effort().durability_volatile());
  current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      current_pose_topic, rclcpp::QoS(1).reliable());

  if (frequency_ <= 0.0) throw std::runtime_error("frequency must be > 0");
  if (command_timeout_ <= 0.0) throw std::runtime_error("command_timeout must be > 0");
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_),
                             std::bind(&PidPositionNode::update, this));

  RCLCPP_INFO(get_logger(),
              "PID Cartesian position controller: %s -> %s at %.1f Hz; watchdog=%.3f s; default I=D=0",
              root_link_.c_str(), tip_link_.c_str(), frequency_, command_timeout_);
}

void PidPositionNode::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (kinematics_ready_) return;
  std::string error;
  if (!kinematics_.initialize(msg->data, root_link_, tip_link_, &error)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
    return;
  }
  kinematics_ready_ = true;
  RCLCPP_INFO(get_logger(), "Kinematics initialized from /robot_description");
}

void PidPositionNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  Vector7d q, dq;
  if (!extractJointState(*msg, joint_names_, q, dq)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  q_ = q;
  dq_ = dq;
  have_joint_state_ = true;
}

void PidPositionNode::desiredPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != root_link_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Desired pose frame '%s' differs from controller root '%s'; no TF transform is applied",
                         msg->header.frame_id.c_str(), root_link_.c_str());
  }
  std::lock_guard<std::mutex> lock(mutex_);
  desired_pose_ = poseMsgToIsometry(msg->pose);
  last_pose_command_time_steady_ = std::chrono::steady_clock::now();
  have_desired_pose_ = true;
  if (reset_integral_on_new_target_) {
    integral_error_.setZero();
    have_previous_error_ = false;
  }
}

void PidPositionNode::update() {
  Vector7d q;
  Eigen::Isometry3d desired;
  bool have_state = false;
  bool have_target = false;
  std::chrono::steady_clock::time_point command_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q = q_;
    desired = desired_pose_;
    have_state = have_joint_state_;
    have_target = have_desired_pose_;
    command_time = last_pose_command_time_steady_;
  }
  if (!kinematics_ready_ || !have_state) return;

  Matrix67d J;
  Eigen::Isometry3d current;
  if (!kinematics_.compute(q, J, current)) return;
  const auto t = now();
  current_pose_pub_->publish(makePoseStamped(current, root_link_, t));

  const auto steady_now = std::chrono::steady_clock::now();
  if (!have_target) {
    last_update_time_steady_ = steady_now;
    have_last_update_time_ = true;
    return;
  }
  const double command_age =
      std::chrono::duration<double>(steady_now - command_time).count();
  if (command_age > command_timeout_) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_desired_pose_ = false;
      integral_error_.setZero();
      have_previous_error_ = false;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Desired pose watchdog expired (age=%.3f > %.3f s); stopping Cartesian commands",
                         command_age, command_timeout_);
    last_update_time_steady_ = steady_now;
    have_last_update_time_ = true;
    return;
  }

  double dt = 1.0 / frequency_;
  if (have_last_update_time_) {
    dt = std::clamp(
        std::chrono::duration<double>(steady_now - last_update_time_steady_).count(),
        1e-4, 0.1);
  }
  last_update_time_steady_ = steady_now;
  have_last_update_time_ = true;

  const Vector6d error = poseError(current, desired);
  Vector6d derivative = Vector6d::Zero();
  if (have_previous_error_) derivative = (error - previous_error_) / dt;
  previous_error_ = error;
  have_previous_error_ = true;

  integral_error_ += error * dt;
  for (int i = 0; i < 6; ++i) {
    const double lim = std::abs(integral_limit_(i));
    integral_error_(i) = std::clamp(integral_error_(i), -lim, lim);
  }

  Vector6d command = kp_.cwiseProduct(error)
                   + ki_.cwiseProduct(integral_error_)
                   + kd_.cwiseProduct(derivative);
  command = saturateAbsPreservingDirection(command, max_twist_);
  if (command.allFinite()) {
    twist_pub_->publish(makeTwistStamped(command, root_link_, t));
  }
}

}  // namespace franka_cartesian_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<franka_cartesian_control::PidPositionNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("pid_position_controller"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
