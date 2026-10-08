#include "franka_cartesian_control/ik_velocity_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>

namespace franka_cartesian_control {
namespace {
Vector7d vec7(const std::vector<double>& v, const char* name) {
  if (v.size() != 7) throw std::runtime_error(std::string(name) + " must contain 7 values");
  Vector7d out;
  for (int i = 0; i < 7; ++i) out(i) = v[i];
  return out;
}
}

IkVelocityNode::IkVelocityNode() : Node("ik_velocity_controller") {
  robot_type_ = declare_parameter<std::string>("robot_type", "fr3");
  arm_prefix_ = declare_parameter<std::string>("arm_prefix", "");
  root_link_ = declare_parameter<std::string>("root_link", "");
  tip_link_ = declare_parameter<std::string>("tip_link", "auto");
  declare_parameter<bool>("load_gripper", false);  // Bringup compatibility; URDF decides the tip.
  const auto default_joint_names = deriveJointNames(robot_type_, arm_prefix_);
  joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names);
  if (joint_names_.size() != 7) throw std::runtime_error("joint_names must contain 7 entries");
  if (root_link_.empty()) root_link_ = deriveRootLink(robot_type_, arm_prefix_);
  // tip_link=auto is resolved from the received /robot_description.
  RCLCPP_INFO(get_logger(), "URDF_TIP_AUTO_V7: requested tip_link=\"%s\"", tip_link_.c_str());

  frequency_ = declare_parameter<double>("frequency", 200.0);
  command_timeout_ = declare_parameter<double>("command_timeout", 0.1);
  pinv_damping_ = declare_parameter<double>("pinv_damping", 0.0001);
  nullspace_gain_ = declare_parameter<double>("nullspace_gain", 0.0);
  joint_limit_margin_ = declare_parameter<double>("joint_limit_margin", 0.08);
  joint_limit_gain_ = declare_parameter<double>("joint_limit_gain", 2.0);
  enforce_surfaces_ = declare_parameter<bool>("enforce_surfaces", true);

  max_velocity_ = vec7(declare_parameter<std::vector<double>>(
      "max_velocity", {1.0,1.0,1.0,1.0,1.5,1.5,1.5}), "max_velocity");
  max_acceleration_ = vec7(declare_parameter<std::vector<double>>(
      "max_acceleration", {0.3,0.3,0.3,0.3,0.3,0.3,0.3}), "max_acceleration");
  q_min_ = vec7(declare_parameter<std::vector<double>>(
      "joint_min", {-2.9007,-1.8361,-2.9007,-3.0770,-2.8763,0.4398,-3.0508}), "joint_min");
  q_max_ = vec7(declare_parameter<std::vector<double>>(
      "joint_max", {2.9007,1.8361,2.9007,-0.1169,2.8763,4.6216,3.0508}), "joint_max");
  nullspace_target_ = vec7(declare_parameter<std::vector<double>>(
      "nullspace_target", {0.0,0.0,0.0,-1.5708,0.0,1.5708,0.7854}), "nullspace_target");

  const auto surface_names = declare_parameter<std::vector<std::string>>("surfaces.names", {"table"});
  const auto surface_normals = declare_parameter<std::vector<double>>("surfaces.normals", {0.0,0.0,1.0});
  const auto surface_offsets = declare_parameter<std::vector<double>>("surfaces.offsets", {0.0});
  const auto surface_margins = declare_parameter<std::vector<double>>("surfaces.margins", {0.08});
  const auto surface_gains = declare_parameter<std::vector<double>>("surfaces.gains", {2.0});
  surface_planes_ = parseSurfacePlanes(surface_names, surface_normals, surface_offsets,
                                       surface_margins, surface_gains, enforce_surfaces_);

  const auto robot_description_topic = declare_parameter<std::string>("robot_description_topic", "/robot_description");
  const auto joint_state_topic = declare_parameter<std::string>("joint_state_topic", "/franka/joint_states");
  const auto twist_topic = declare_parameter<std::string>("twist_topic", "/fr3/cartesian_twist_command");
  const auto command_topic = declare_parameter<std::string>("joint_command_topic", "/fr3/joint_commands");

  auto robot_qos = rclcpp::QoS(1).transient_local().reliable();
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic, robot_qos,
      std::bind(&IkVelocityNode::robotDescriptionCallback, this, std::placeholders::_1));
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      std::bind(&IkVelocityNode::jointStateCallback, this, std::placeholders::_1));
  twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic, rclcpp::QoS(1).best_effort().durability_volatile(),
      std::bind(&IkVelocityNode::twistCallback, this, std::placeholders::_1));
  command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      command_topic, rclcpp::QoS(1).best_effort().durability_volatile());
  ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "~/ready", rclcpp::QoS(1).transient_local().reliable());

  if (frequency_ <= 0.0) throw std::runtime_error("frequency must be > 0");
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_),
                             std::bind(&IkVelocityNode::update, this));
  RCLCPP_INFO(get_logger(), "IK velocity controller: %s -> %s at %.1f Hz", root_link_.c_str(),
              (tip_link_ == "auto" ? "<auto from URDF>" : tip_link_.c_str()), frequency_);
}

void IkVelocityNode::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (kinematics_ready_) return;
  std::string error;
  if (!kinematics_.initializeFromRobotDescriptionV7(msg->data, root_link_, tip_link_, &error)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
    return;
  }
  tip_link_ = kinematics_.tipLink();
  kinematics_ready_ = true;
  RCLCPP_INFO(get_logger(), "FR3_KINEMATICS_V7: initialized from /robot_description: %s -> %s",
              root_link_.c_str(), tip_link_.c_str());
}

void IkVelocityNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  Vector7d q, dq;
  if (extractJointState(*msg, joint_names_, q, dq)) {
    q_ = q; dq_ = dq;
    if (!have_joint_state_) previous_command_ = dq;
    have_joint_state_ = true;
  }
}

void IkVelocityNode::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  const auto x = twistToVector(*msg);
  if (!x.allFinite()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  desired_twist_ = x;
  last_twist_time_steady_ = std::chrono::steady_clock::now();
  have_twist_ = true;
}

void IkVelocityNode::update() {
  Vector7d q, dq, previous;
  Vector6d twist = Vector6d::Zero();
  std::chrono::steady_clock::time_point last_twist;
  bool have_state, have_twist;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q = q_; dq = dq_; previous = previous_command_;
    twist = desired_twist_; last_twist = last_twist_time_steady_;
    have_state = have_joint_state_; have_twist = have_twist_;
  }
  if (kinematics_ready_ && have_state && !ready_published_) {
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);
    ready_published_ = true;
    RCLCPP_INFO(get_logger(), "IK controller is operational (kinematics + joint state ready)");
  }
  if (!kinematics_ready_ || !have_state) return;

  const auto t = now();
  const auto steady_now = std::chrono::steady_clock::now();

  // No Cartesian command means no joint command.  This is important when the
  // downstream controller is impedance/effort based: continuously publishing
  // dq_des = 0 would actively damp the robot and make it feel artificially stiff.
  if (!have_twist) {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_command_ = dq;
    last_update_time_steady_ = steady_now;
    have_last_update_time_ = true;
    return;
  }

  // Timeout braking is modeled here using the same high-level acceleration
  // constraint as normal IK motion. The low-level controller then interpolates
  // these targets at 1 kHz so Franka sees continuous acceleration and jerk.
  if (std::chrono::duration<double>(steady_now - last_twist).count() > command_timeout_) {
    double stop_dt = 1.0 / frequency_;
    if (have_last_update_time_) {
      stop_dt = std::clamp(
          std::chrono::duration<double>(steady_now - last_update_time_steady_).count(),
          1e-4, 0.05);
    }
    Vector7d stop_command = saturateAccelerationPreservingDirection(
        Vector7d::Zero(), previous, max_acceleration_, stop_dt);
    if (stop_command.norm() < 1e-4) stop_command.setZero();
    command_pub_->publish(makeVelocityCommand(joint_names_, stop_command, t));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      desired_twist_.setZero();
      previous_command_ = stop_command;
      last_update_time_steady_ = steady_now;
      have_last_update_time_ = true;
      if (stop_command.isZero(1e-6)) have_twist_ = false;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Cartesian command timed out; ramping IK joint velocity to zero");
    return;
  }

  double dt = 1.0 / frequency_;
  if (have_last_update_time_) {
    dt = std::clamp(
        std::chrono::duration<double>(steady_now - last_update_time_steady_).count(),
        1e-4, 0.05);
  }
  last_update_time_steady_ = steady_now;
  have_last_update_time_ = true;

  Matrix67d J;
  Eigen::Isometry3d pose;
  if (!kinematics_.compute(q, J, pose)) return;

  const auto Jpinv = dampedPseudoInverse(J, pinv_damping_);
  Vector7d qdot = Jpinv * twist;
  if (nullspace_gain_ > 0.0) {
    const Eigen::Matrix<double,7,7> N = Eigen::Matrix<double,7,7>::Identity() - Jpinv * J;
    qdot += N * (nullspace_gain_ * (nullspace_target_ - q));
  }

  Vector7d lower, upper;
  combineJointBounds(q, max_velocity_, q_min_, q_max_, joint_limit_margin_, joint_limit_gain_, lower, upper);

  // Do not clip joints independently: that changes J*qdot and bends a requested
  // straight Cartesian motion. Find the single worst bound violation and scale
  // the complete joint vector by the same factor instead.
  double velocity_scale = 1.0;
  bool origin_feasible = true;
  qdot = saturateVectorPreservingDirection(
      qdot, lower, upper, &velocity_scale, &origin_feasible);

  // Apply acceleration limits to the complete change vector with one common
  // scale factor. This preserves the direction of the requested acceleration
  // rather than clipping individual joints.
  double acceleration_scale = 1.0;
  qdot = saturateAccelerationPreservingDirection(
      qdot, previous, max_acceleration_, dt, &acceleration_scale);

  RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "IK saturation: velocity_scale=%.3f acceleration_scale=%.3f origin_feasible=%d",
      velocity_scale, acceleration_scale, origin_feasible ? 1 : 0);

  // TCP half-space constraints: n^T p >= offset + margin. While still on the
  // valid side, satisfy the CBF by scaling the whole command toward zero, which
  // preserves Cartesian direction. If the plane has already been crossed, an
  // outward recovery direction is required and cannot be represented by a pure
  // scalar saturation, so use the minimum-norm recovery correction.
  for (int pass = 0; pass < 2; ++pass) {
    for (const auto& plane : surface_planes_) {
      const Eigen::Matrix<double,1,7> a = plane.normal.transpose() * J.topRows<3>();
      const double distance = plane.normal.dot(pose.translation()) - (plane.offset + plane.margin);
      const double lower_speed = -plane.gain * distance;
      const double actual = (a * qdot)(0);
      const double denom = a.squaredNorm();
      if (actual >= lower_speed || denom <= 1e-10) continue;

      if (distance >= 0.0 && actual < 0.0 && lower_speed <= 0.0) {
        const double surface_scale = std::clamp(lower_speed / actual, 0.0, 1.0);
        qdot *= surface_scale;
      } else {
        qdot += a.transpose() * ((lower_speed - actual) / denom);
        qdot = saturateVectorPreservingDirection(qdot, lower, upper);
      }
    }
  }

  if (!qdot.allFinite()) qdot.setZero();
  command_pub_->publish(makeVelocityCommand(joint_names_, qdot, t));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_command_ = qdot;
  }
}

void IkVelocityNode::publishZero() {
  command_pub_->publish(makeVelocityCommand(joint_names_, Vector7d::Zero(), now()));
}

}  // namespace franka_cartesian_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<franka_cartesian_control::IkVelocityNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ik_velocity_controller"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
