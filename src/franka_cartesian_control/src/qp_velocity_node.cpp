#include "franka_cartesian_control/qp_velocity_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>

namespace franka_cartesian_control {
namespace {
Vector7d vec7(const std::vector<double>& v, const char* name) {
  if (v.size() != 7) throw std::runtime_error(std::string(name) + " must contain 7 values");
  Vector7d out; for (int i = 0; i < 7; ++i) out(i) = v[i]; return out;
}
Vector6d vec6(const std::vector<double>& v, const char* name) {
  if (v.size() != 6) throw std::runtime_error(std::string(name) + " must contain 6 values");
  Vector6d out; for (int i = 0; i < 6; ++i) out(i) = v[i]; return out;
}
}

QpVelocityNode::QpVelocityNode() : Node("qp_velocity_controller") {
  robot_type_ = declare_parameter<std::string>("robot_type", "fr3");
  arm_prefix_ = declare_parameter<std::string>("arm_prefix", "");
  root_link_ = declare_parameter<std::string>("root_link", "");
  tip_link_ = declare_parameter<std::string>("tip_link", "");
  const auto default_joint_names = deriveJointNames(robot_type_, arm_prefix_);
  joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", default_joint_names);
  if (joint_names_.size() != 7) throw std::runtime_error("joint_names must contain 7 entries");
  if (root_link_.empty()) root_link_ = deriveRootLink(robot_type_, arm_prefix_);
  tip_link_ = deriveTipLink(robot_type_, arm_prefix_, tip_link_);

  frequency_ = declare_parameter<double>("frequency", 200.0);
  command_timeout_ = declare_parameter<double>("command_timeout", 0.1);
  idle_safety_frequency_ = declare_parameter<double>("idle_safety_frequency", 50.0);
  repulsion_when_idle_ = declare_parameter<bool>("repulsion_when_idle", true);
  joint_limit_margin_ = declare_parameter<double>("joint_limit_margin", 0.01);
  joint_limit_gain_ = declare_parameter<double>("joint_limit_gain", 1.0);
  joint_repulsion_gain_ = declare_parameter<double>("joint_repulsion_gain", 0.2);
  joint_repulsion_weight_ = declare_parameter<double>("joint_repulsion_weight", 0.25);
  max_joint_repulsion_velocity_ = declare_parameter<double>("max_joint_repulsion_velocity", 0.35);
  joint_recovery_release_margin_ = declare_parameter<double>("joint_recovery_release_margin", 0.03);
  recovery_command_time_constant_ = declare_parameter<double>("recovery_command_time_constant", 0.12);
  recovery_release_tolerance_ = declare_parameter<double>("recovery_release_tolerance", 0.01);
  regularization_ = declare_parameter<double>("regularization", 1e-4);
  pinv_damping_ = declare_parameter<double>("pinv_damping", 0.000001);
  posture_weight_ = declare_parameter<double>("posture_weight", 0.02);
  posture_gain_ = declare_parameter<double>("posture_gain", 0.1);
  max_working_set_recalculations_ = declare_parameter<int>("max_working_set_recalculations", 100);
  enforce_surfaces_ = declare_parameter<bool>("enforce_surfaces", true);

  max_velocity_ = vec7(declare_parameter<std::vector<double>>(
      "max_velocity", {1.0,1.0,1.0,1.0,1.5,1.5,1.5}), "max_velocity");
  max_acceleration_ = vec7(declare_parameter<std::vector<double>>(
      "max_acceleration", {0.3,0.3,0.3,0.3,0.3,0.3,0.3}), "max_acceleration");
  q_min_ = vec7(declare_parameter<std::vector<double>>(
      "joint_min", {-2.9007,-1.8361,-2.9007,-3.0770,-2.8763,0.4398,-3.0508}), "joint_min");
  q_max_ = vec7(declare_parameter<std::vector<double>>(
      "joint_max", {2.9007,1.8361,2.9007,-0.1169,2.8763,4.6216,3.0508}), "joint_max");
  posture_target_ = vec7(declare_parameter<std::vector<double>>(
      "posture_target", {0.0,0.0,0.0,-1.5708,0.0,1.5708,0.7854}), "posture_target");
  task_weights_ = vec6(declare_parameter<std::vector<double>>(
      "task_weights", {1.0,1.0,1.0,1.0,1.0,1.0}), "task_weights");

  const auto surface_names = declare_parameter<std::vector<std::string>>("surfaces.names", {"table"});
  const auto surface_normals = declare_parameter<std::vector<double>>("surfaces.normals", {0.0,0.0,1.0});
  const auto surface_offsets = declare_parameter<std::vector<double>>("surfaces.offsets", {0.0});
  const auto surface_margins = declare_parameter<std::vector<double>>("surfaces.margins", {0.08});
  const auto surface_gains = declare_parameter<std::vector<double>>("surfaces.gains", {2.0});
  surface_repulsion_gains_ = declare_parameter<std::vector<double>>("surfaces.repulsion_gains", {3.0});
  surface_repulsion_weights_ = declare_parameter<std::vector<double>>("surfaces.repulsion_weights", {3.0});
  surface_max_repulsion_velocities_ = declare_parameter<std::vector<double>>(
      "surfaces.max_repulsion_velocities", {0.05});
  surface_release_distances_ = declare_parameter<std::vector<double>>(
      "surfaces.release_distances", {0.015});
  surface_planes_ = parseSurfacePlanes(surface_names, surface_normals, surface_offsets,
                                       surface_margins, surface_gains, enforce_surfaces_);
  if (enforce_surfaces_ &&
      (surface_repulsion_gains_.size() != surface_planes_.size() ||
       surface_repulsion_weights_.size() != surface_planes_.size() ||
       surface_max_repulsion_velocities_.size() != surface_planes_.size() ||
       surface_release_distances_.size() != surface_planes_.size())) {
    throw std::runtime_error(
        "surface repulsion/release arrays must each contain one value per enabled surface");
  }

  const auto robot_description_topic = declare_parameter<std::string>("robot_description_topic", "/robot_description");
  const auto joint_state_topic = declare_parameter<std::string>("joint_state_topic", "/franka/joint_states");
  const auto twist_topic = declare_parameter<std::string>("twist_topic", "/fr3/cartesian_twist_command");
  const auto command_topic = declare_parameter<std::string>("joint_command_topic", "/fr3/joint_commands");

  auto robot_qos = rclcpp::QoS(1).transient_local().reliable();
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic, robot_qos,
      std::bind(&QpVelocityNode::robotDescriptionCallback, this, std::placeholders::_1));
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      std::bind(&QpVelocityNode::jointStateCallback, this, std::placeholders::_1));
  twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic, rclcpp::QoS(1).best_effort().durability_volatile(),
      std::bind(&QpVelocityNode::twistCallback, this, std::placeholders::_1));
  command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      command_topic, rclcpp::QoS(1).best_effort().durability_volatile());
  status_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/status", 10);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "~/ready", rclcpp::QoS(1).transient_local().reliable());

  qp_ = std::make_unique<qpOASES::SQProblem>(7, static_cast<int>(surface_planes_.size()));
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp_->setOptions(options);

  if (frequency_ <= 0.0) throw std::runtime_error("frequency must be > 0");
  if (joint_recovery_release_margin_ < 0.0 || recovery_command_time_constant_ <= 0.0 ||
      recovery_release_tolerance_ < 0.0) {
    throw std::runtime_error("recovery release parameters are invalid");
  }
  for (double d : surface_release_distances_) {
    if (d < 0.0) throw std::runtime_error("surfaces.release_distances must be >= 0");
  }
  surface_recovery_active_.assign(surface_planes_.size(), false);
  if (idle_safety_frequency_ <= 0.0) throw std::runtime_error("idle_safety_frequency must be > 0");
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_),
                             std::bind(&QpVelocityNode::update, this));
  RCLCPP_INFO(get_logger(), "QP velocity controller: %s -> %s, %zu surface constraints, %.1f Hz",
              root_link_.c_str(), tip_link_.c_str(), surface_planes_.size(), frequency_);
}

void QpVelocityNode::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (kinematics_ready_) return;
  std::string error;
  if (!kinematics_.initialize(msg->data, root_link_, tip_link_, &error)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
    return;
  }
  kinematics_ready_ = true;
  RCLCPP_INFO(get_logger(), "Kinematics initialized from /robot_description");
}

void QpVelocityNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  Vector7d q, dq;
  if (extractJointState(*msg, joint_names_, q, dq)) {
    q_ = q; dq_ = dq;
    if (!have_joint_state_) previous_command_ = dq;
    have_joint_state_ = true;
  }
}

void QpVelocityNode::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  const auto x = twistToVector(*msg);
  if (!x.allFinite()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  desired_twist_ = x;
  last_twist_time_steady_ = std::chrono::steady_clock::now();
  have_twist_ = true;
}

void QpVelocityNode::update() {
  Vector7d q, dq;
  Vector6d twist;
  std::chrono::steady_clock::time_point last_twist;
  bool have_state, have_twist;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q = q_; dq = dq_; twist = desired_twist_; last_twist = last_twist_time_steady_;
    have_state = have_joint_state_; have_twist = have_twist_;
  }
  if (kinematics_ready_ && have_state && !ready_published_) {
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_pub_->publish(ready);
    ready_published_ = true;
    RCLCPP_INFO(get_logger(), "QP controller is operational (kinematics + joint state ready)");
  }
  if (!kinematics_ready_ || !have_state) return;

  const auto t = now();
  const auto steady_now = std::chrono::steady_clock::now();
  const bool command_fresh = have_twist &&
      (std::chrono::duration<double>(steady_now - last_twist).count() <= command_timeout_);

  if (command_fresh) {
    stopping_on_timeout_ = false;
  } else if (have_twist) {
    std::lock_guard<std::mutex> lock(mutex_);
    have_twist_ = false;
    desired_twist_.setZero();
    have_twist = false;
    stopping_on_timeout_ = true;
  }

  double dt = 1.0 / frequency_;
  if (have_last_update_time_) {
    dt = std::clamp(
        std::chrono::duration<double>(steady_now - last_update_time_steady_).count(),
        1e-4, 0.05);
  }
  last_update_time_steady_ = steady_now;
  have_last_update_time_ = true;

  // Decelerate the target to zero inside the QP after a twist timeout. The
  // zero-twist task plus acceleration bounds produces a bounded high-level stop;
  // the low-level layer only interpolates that target at 1 kHz for Franka-safe
  // acceleration/jerk continuity. Posture is disabled for this stop below.
  if (!command_fresh && stopping_on_timeout_) {
    Matrix67d stop_J;
    Eigen::Isometry3d stop_pose;
    if (!kinematics_.compute(q, stop_J, stop_pose)) return;
    updateJointRecoveryState(q);
    updateSurfaceRecoveryState(stop_pose);
    Vector7d stop_solution = Vector7d::Zero();
    if (solveQp(stop_J, stop_pose, Vector6d::Zero(), dt, stop_solution, true)) {
      if (stop_solution.norm() < 1e-4) stop_solution.setZero();
      command_pub_->publish(makeVelocityCommand(joint_names_, stop_solution, t));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        previous_command_ = stop_solution;
      }
      if (stop_solution.isZero(1e-6) && dq.norm() < 1e-3) {
        stopping_on_timeout_ = false;
      }
      return;
    }
  }

  // Without a user command, stay idle unless a configured safety boundary has
  // actually been crossed. Once recovery activates it is latched until the state
  // has returned a small configurable distance inside the valid region. This
  // hysteresis prevents chatter and gives the robot a real "push back" motion.
  bool need_recovery = false;
  bool release_blending = false;
  Matrix67d J;
  Eigen::Isometry3d pose;
  bool have_kinematics = false;

  if (!command_fresh) {
    if (!repulsion_when_idle_) {
      std::lock_guard<std::mutex> lock(mutex_);
      previous_command_ = dq;
      recovery_command_ = dq;
      last_update_time_steady_ = steady_now;
      have_last_update_time_ = true;
      recovery_active_ = false;
      return;
    }

    need_recovery = updateJointRecoveryState(q);

    const bool safety_check_due = recovery_active_ || anyRecoveryStateActive() ||
      !have_last_idle_safety_check_time_ ||
      std::chrono::duration<double>(steady_now - last_idle_safety_check_time_steady_).count() >=
          (1.0 / idle_safety_frequency_);

    if (safety_check_due && enforce_surfaces_ && !surface_planes_.empty()) {
      if (kinematics_.compute(q, J, pose)) {
        have_kinematics = true;
        last_idle_safety_check_time_steady_ = steady_now;
        have_last_idle_safety_check_time_ = true;
        need_recovery = updateSurfaceRecoveryState(pose) || need_recovery;
      }
    }

    if (!need_recovery) {
      if (recovery_active_) {
        qp_->reset();
        qp_initialized_ = false;
      }
      recovery_active_ = false;

      // Do not drop the velocity-feedback command abruptly. Blend the command
      // toward the measured joint velocity, which makes dq_des-dq -> 0 smoothly.
      // Once the error is negligible we stop publishing and the low-level
      // impedance timeout becomes torque-continuous.
      if ((recovery_command_ - dq).norm() > recovery_release_tolerance_) {
        release_blending = true;
      } else {
        recovery_command_ = dq;
        std::lock_guard<std::mutex> lock(mutex_);
        previous_command_ = dq;
        last_update_time_steady_ = steady_now;
        have_last_update_time_ = true;
        return;
      }
    } else {
      recovery_active_ = true;
      twist.setZero();
    }
  }

  if (release_blending) {
    const double alpha = std::clamp(dt / (recovery_command_time_constant_ + dt), 0.0, 1.0);
    recovery_command_ += alpha * (dq - recovery_command_);
    command_pub_->publish(makeVelocityCommand(joint_names_, recovery_command_, t));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      previous_command_ = recovery_command_;
    }
    return;
  }

  if (!have_kinematics) {
    if (!kinematics_.compute(q, J, pose)) return;
  }

  Vector7d solution = Vector7d::Zero();
  if (!solveQp(J, pose, twist, dt, solution, command_fresh)) {
    Vector7d lower, upper;
    combineJointBounds(q, max_velocity_, q_min_, q_max_, joint_limit_margin_, joint_limit_gain_, lower, upper);
    applyAccelerationBounds(previous_command_, max_acceleration_, dt, lower, upper);
    solution = clampVector(Vector7d::Zero(), lower, upper);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "QP solve failed; ramping command toward zero");
  }
  if (!solution.allFinite()) solution.setZero();

  if (!command_fresh && recovery_active_ && enforce_surfaces_) {
    for (std::size_t c = 0; c < surface_planes_.size(); ++c) {
      if (!surface_recovery_active_[c]) continue;
      const auto& plane = surface_planes_[c];
      const Eigen::Matrix<double,1,7> a =
          plane.normal.transpose() * J.topRows<3>();
      const double distance =
          plane.normal.dot(pose.translation()) - (plane.offset + plane.margin);
      const double measured_normal_velocity = (a * dq)(0);
      const double commanded_normal_velocity = (a * solution)(0);
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 250,
          "Surface recovery '%s': distance=%.4f m, measured normal v=%.4f m/s, commanded normal v=%.4f m/s",
          plane.name.c_str(), distance, measured_normal_velocity, commanded_normal_velocity);
    }
  }

  if (!command_fresh) {
    // IMPORTANT: do not low-pass filter an active recovery solution here.
    // The QP solution already satisfies the surface/joint constraints and its
    // acceleration bounds provide the required command smoothing. Filtering the
    // solution after the QP can move it back outside the feasible set (and, when
    // hand-guiding inward, can re-introduce an inward normal velocity). Keep the
    // exact feasible recovery command while recovery is active.
    recovery_command_ = solution;
  }

  command_pub_->publish(makeVelocityCommand(joint_names_, solution, t));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    previous_command_ = solution;
  }
}

bool QpVelocityNode::updateJointRecoveryState(const Vector7d& q) {
  bool active = false;
  for (int i = 0; i < 7; ++i) {
    const double lo_safe = q_min_(i) + joint_limit_margin_;
    const double hi_safe = q_max_(i) - joint_limit_margin_;

    if (joint_recovery_direction_[i] == 0) {
      if (q(i) < lo_safe) joint_recovery_direction_[i] = +1;
      else if (q(i) > hi_safe) joint_recovery_direction_[i] = -1;
    } else if (joint_recovery_direction_[i] > 0) {
      if (q(i) >= lo_safe + joint_recovery_release_margin_) {
        joint_recovery_direction_[i] = 0;
      }
    } else {
      if (q(i) <= hi_safe - joint_recovery_release_margin_) {
        joint_recovery_direction_[i] = 0;
      }
    }
    active = active || (joint_recovery_direction_[i] != 0);
  }
  return active;
}

bool QpVelocityNode::updateSurfaceRecoveryState(const Eigen::Isometry3d& pose) {
  if (!enforce_surfaces_) return false;
  bool active = false;
  for (std::size_t c = 0; c < surface_planes_.size(); ++c) {
    const auto& plane = surface_planes_[c];
    const double distance =
        plane.normal.dot(pose.translation()) - (plane.offset + plane.margin);
    if (!surface_recovery_active_[c]) {
      if (distance < 0.0) surface_recovery_active_[c] = true;
    } else if (distance >= surface_release_distances_[c]) {
      surface_recovery_active_[c] = false;
    }
    active = active || surface_recovery_active_[c];
  }
  return active;
}

bool QpVelocityNode::anyRecoveryStateActive() const {
  for (int d : joint_recovery_direction_) {
    if (d != 0) return true;
  }
  return std::any_of(surface_recovery_active_.begin(), surface_recovery_active_.end(),
                     [](bool v) { return v; });
}

bool QpVelocityNode::solveQp(const Matrix67d& J, const Eigen::Isometry3d& pose,
                             const Vector6d& twist, double dt, Vector7d& solution,
                             bool tracking_enabled) {
  Eigen::Matrix<double,6,6> W = task_weights_.asDiagonal();
  const Eigen::Matrix<double,7,6> J_pinv = dampedPseudoInverse(J, pinv_damping_);
  const Eigen::Matrix<double,7,7> N =
      Eigen::Matrix<double,7,7>::Identity() - J_pinv * J;

  // With an active user command, Cartesian velocity is the primary task.
  // During idle safety recovery there is deliberately no zero-velocity Cartesian
  // objective; only the violated-limit recovery terms and hard constraints act.
  Eigen::Matrix<double,7,7> H = Eigen::Matrix<double,7,7>::Zero();
  Vector7d g = Vector7d::Zero();
  if (tracking_enabled) {
    H += 2.0 * (J.transpose() * W * W * J);
    g += -2.0 * J.transpose() * W * W * twist;
  }

  // Secondary posture regulation exists only while tracking a user command.
  if (tracking_enabled && twist.squaredNorm() > 1e-12 &&
      posture_weight_ > 0.0 && posture_gain_ > 0.0) {
    const Vector7d posture_velocity = posture_gain_ * (posture_target_ - q_);
    const Eigen::Matrix<double,7,7> NtN = N.transpose() * N;
    H += 2.0 * posture_weight_ * NtN;
    g += -2.0 * posture_weight_ * NtN * posture_velocity;
  }

  // Soft recovery objectives activate only AFTER a configured limit has been crossed.
  // They are exactly zero in the valid region, so they do not bias nominal motion.
  // Hard bounds/inequalities below remain the actual safety layer.
  for (int i = 0; i < 7; ++i) {
    const double lo_safe = q_min_(i) + joint_limit_margin_;
    const double hi_safe = q_max_(i) - joint_limit_margin_;
    double recovery_velocity = 0.0;
    if (joint_recovery_direction_[i] > 0) {
      const double remaining = std::max(0.0,
          (lo_safe + joint_recovery_release_margin_) - q_(i));
      recovery_velocity = std::min(
          max_joint_repulsion_velocity_, joint_repulsion_gain_ * remaining);
    } else if (joint_recovery_direction_[i] < 0) {
      const double remaining = std::max(0.0,
          q_(i) - (hi_safe - joint_recovery_release_margin_));
      recovery_velocity = -std::min(
          max_joint_repulsion_velocity_, joint_repulsion_gain_ * remaining);
    }
    if (recovery_velocity != 0.0 && joint_repulsion_weight_ > 0.0) {
      H(i, i) += 2.0 * joint_repulsion_weight_;
      g(i) += -2.0 * joint_repulsion_weight_ * recovery_velocity;
    }
  }

  for (std::size_t c = 0; c < surface_planes_.size(); ++c) {
    const auto& plane = surface_planes_[c];
    const double distance =
        plane.normal.dot(pose.translation()) - (plane.offset + plane.margin);
    if (surface_recovery_active_[c] && surface_repulsion_weights_[c] > 0.0) {
      const Eigen::Matrix<double,1,7> a =
          plane.normal.transpose() * J.topRows<3>();
      const double remaining = std::max(0.0, surface_release_distances_[c] - distance);
      const double recovery_speed = std::min(
          surface_max_repulsion_velocities_[c],
          surface_repulsion_gains_[c] * remaining);
      H += 2.0 * surface_repulsion_weights_[c] * (a.transpose() * a);
      g += -2.0 * surface_repulsion_weights_[c] * recovery_speed * a.transpose();
    }
  }

  // Small full-space regularization for numerical conditioning only.
  H.diagonal().array() += 2.0 * regularization_;

  Vector7d lower, upper;
  combineJointBounds(q_, max_velocity_, q_min_, q_max_, joint_limit_margin_, joint_limit_gain_, lower, upper);
  applyAccelerationBounds(previous_command_, max_acceleration_, dt, lower, upper);

  const int nC = static_cast<int>(surface_planes_.size());
  Eigen::Matrix<double, Eigen::Dynamic, 7, Eigen::RowMajor> A(nC, 7);
  Eigen::VectorXd lbA(nC), ubA(nC);
  for (int c = 0; c < nC; ++c) {
    const auto idx = static_cast<std::size_t>(c);
    const auto& plane = surface_planes_[idx];
    A.row(c) = plane.normal.transpose() * J.topRows<3>();
    const double distance =
        plane.normal.dot(pose.translation()) - (plane.offset + plane.margin);

    // CBF-style hard surface constraint. When the surface has already been
    // crossed, -gain*distance asks for an outward recovery velocity. However,
    // that requested velocity can be unreachable in one QP cycle because the
    // joint acceleration bounds above restrict how far qdot may move from the
    // previous command. Requiring an unreachable Cartesian velocity makes the
    // QP infeasible exactly when recovery is needed.
    //
    // Cap the hard lower bound at the maximum normal velocity that is actually
    // achievable inside the current joint-velocity box. The soft recovery cost
    // above still asks for the full recovery velocity, so the optimizer moves
    // outward as quickly as the acceleration limits allow over successive cycles.
    const double requested_lb = -plane.gain * distance;
    double max_achievable_normal_velocity = 0.0;
    for (int j = 0; j < 7; ++j) {
      const double a_j = A(c, j);
      max_achievable_normal_velocity +=
          a_j >= 0.0 ? a_j * upper(j) : a_j * lower(j);
    }
    lbA(c) = std::min(requested_lb, max_achievable_normal_velocity);
    ubA(c) = 1.0e6;
  }

  std::array<qpOASES::real_t, 49> Hq{};
  std::array<qpOASES::real_t, 7> gq{}, lb{}, ub{}, x{};
  for (int r = 0; r < 7; ++r) {
    gq[r] = g(r); lb[r] = lower(r); ub[r] = upper(r);
    for (int c = 0; c < 7; ++c) Hq[7*r+c] = H(r,c);
  }
  std::vector<qpOASES::real_t> Aq(static_cast<std::size_t>(nC * 7));
  std::vector<qpOASES::real_t> lA(static_cast<std::size_t>(nC));
  std::vector<qpOASES::real_t> uA(static_cast<std::size_t>(nC));
  for (int r = 0; r < nC; ++r) {
    lA[r] = lbA(r); uA[r] = ubA(r);
    for (int c = 0; c < 7; ++c) Aq[7*r+c] = A(r,c);
  }

  int nWSR = max_working_set_recalculations_;
  qpOASES::returnValue ret;
  auto* Aptr = nC > 0 ? Aq.data() : nullptr;
  auto* lAptr = nC > 0 ? lA.data() : nullptr;
  auto* uAptr = nC > 0 ? uA.data() : nullptr;
  if (!qp_initialized_) {
    ret = qp_->init(Hq.data(), gq.data(), Aptr, lb.data(), ub.data(), lAptr, uAptr, nWSR, nullptr);
    qp_initialized_ = (ret == qpOASES::SUCCESSFUL_RETURN);
  } else {
    ret = qp_->hotstart(Hq.data(), gq.data(), Aptr, lb.data(), ub.data(), lAptr, uAptr, nWSR, nullptr);
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
      qp_->reset();
      qp_initialized_ = false;
      nWSR = max_working_set_recalculations_ * 2;
      ret = qp_->init(Hq.data(), gq.data(), Aptr, lb.data(), ub.data(), lAptr, uAptr, nWSR, nullptr);
      qp_initialized_ = (ret == qpOASES::SUCCESSFUL_RETURN);
    }
  }
  if (ret != qpOASES::SUCCESSFUL_RETURN || !qp_initialized_) return false;
  if (qp_->getPrimalSolution(x.data()) != qpOASES::SUCCESSFUL_RETURN) return false;
  for (int i = 0; i < 7; ++i) solution(i) = x[i];

  std_msgs::msg::Float64MultiArray status;
  status.data = {static_cast<double>(ret), static_cast<double>(nWSR)};
  status_pub_->publish(status);
  return true;
}

}  // namespace franka_cartesian_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<franka_cartesian_control::QpVelocityNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("qp_velocity_controller"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
