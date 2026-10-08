#include "franka_cartesian_control/lmpc_position_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "franka_cartesian_control/position_control_common.hpp"

namespace franka_cartesian_control {
namespace {

Eigen::Isometry3d markerPoseToIsometry(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() << pose.position.x, pose.position.y, pose.position.z;
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                       pose.orientation.y, pose.orientation.z);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  transform.linear() = q.toRotationMatrix();
  return transform;
}

bool allNonNegative(const Vector6d& value) {
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(value(i)) || value(i) < 0.0) return false;
  }
  return true;
}

bool allPositive(const Vector6d& value) {
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(value(i)) || value(i) <= 0.0) return false;
  }
  return true;
}

}  // namespace

LmpcPositionNode::LmpcPositionNode() : Node("lmpc_position_controller") {
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
  horizon_ = declare_parameter<int>("horizon", 15);
  max_nwsr_ = declare_parameter<int>("max_working_set_recalculations", 100);
  if (frequency_ <= 0.0) throw std::runtime_error("frequency must be > 0");
  if (command_timeout_ <= 0.0) throw std::runtime_error("command_timeout must be > 0");
  if (horizon_ < 1 || horizon_ > 100) throw std::runtime_error("horizon must be in [1, 100]");

  state_weights_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "state_weights", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}), "state_weights");
  terminal_weights_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "terminal_weights", {30.0, 30.0, 30.0, 20.0, 20.0, 20.0}), "terminal_weights");
  velocity_weights_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "velocity_weights", {1.0, 1.0, 1.0, 0.50, 0.50, 0.50}), "velocity_weights");
  terminal_velocity_weights_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "terminal_velocity_weights", {50.0, 50.0, 50.0, 20.0, 20.0, 20.0}),
      "terminal_velocity_weights");
  input_weights_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "input_weights", {0.01, 0.01, 0.01, 0.01, 0.01, 0.01}), "input_weights");
  regularization_gain_ = declare_parameter<double>("regularization_gain", 0.01);
  delta_twist_weight_ = declare_parameter<double>("delta_twist_weight", 0.5);
  velocity_tracking_time_constant_ =
      declare_parameter<double>("velocity_tracking_time_constant", 0.08);
  if (regularization_gain_ < 0.0 || delta_twist_weight_ < 0.0) {
    throw std::runtime_error("regularization_gain and delta_twist_weight must be >= 0");
  }
  if (!std::isfinite(velocity_tracking_time_constant_) ||
      velocity_tracking_time_constant_ <= 0.0) {
    throw std::runtime_error("velocity_tracking_time_constant must be finite and > 0");
  }
  // Preserve the scalar tuning interface: regularization_gain uniformly
  // penalizes the desired Cartesian twist command over the horizon.
  input_weights_.setConstant(regularization_gain_);

  max_twist_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "max_twist", {0.50, 0.50, 0.50, 0.20, 0.20, 0.20}), "max_twist");
  max_acceleration_ = vector6FromStd(declare_parameter<std::vector<double>>(
      "max_acceleration", {2.0, 2.0, 2.0, 4.0, 4.0, 4.0}), "max_acceleration");
  if (!allNonNegative(state_weights_) || !allNonNegative(terminal_weights_) ||
      !allNonNegative(velocity_weights_) || !allNonNegative(terminal_velocity_weights_) ||
      !allNonNegative(input_weights_)) {
    throw std::runtime_error("LMPC weights must be finite and >= 0");
  }
  if (!allPositive(max_twist_) || !allPositive(max_acceleration_)) {
    throw std::runtime_error("max_twist and max_acceleration must be finite and > 0");
  }

  obstacle_avoidance_enabled_ = declare_parameter<bool>("obstacles.enabled", true);
  obstacle_safety_distance_ = declare_parameter<double>("obstacles.safety_distance", 0.05);
  obstacle_influence_distance_ = declare_parameter<double>("obstacles.influence_distance", 0.15);
  obstacle_barrier_gain_ = declare_parameter<double>("obstacles.barrier_gain", 3.0);
  obstacle_max_active_meshes_ = declare_parameter<int>("obstacles.max_active_meshes", 16);
  default_table_enabled_ = declare_parameter<bool>("obstacles.default_table.enabled", true);
  default_table_id_ = declare_parameter<int>("obstacles.default_table.id", 0);
  default_table_z_ = declare_parameter<double>("obstacles.default_table.z", 0.0);
  default_table_x_min_ = declare_parameter<double>("obstacles.default_table.x_min", -1.0);
  default_table_x_max_ = declare_parameter<double>("obstacles.default_table.x_max", 1.0);
  default_table_y_min_ = declare_parameter<double>("obstacles.default_table.y_min", -1.0);
  default_table_y_max_ = declare_parameter<double>("obstacles.default_table.y_max", 1.0);
  if (obstacle_safety_distance_ < 0.0) throw std::runtime_error("obstacles.safety_distance must be >= 0");
  if (obstacle_influence_distance_ < obstacle_safety_distance_) {
    throw std::runtime_error("obstacles.influence_distance must be >= obstacles.safety_distance");
  }
  if (obstacle_barrier_gain_ <= 0.0) throw std::runtime_error("obstacles.barrier_gain must be > 0");
  if (obstacle_max_active_meshes_ < 1) throw std::runtime_error("obstacles.max_active_meshes must be >= 1");

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
  const auto obstacle_mesh_topic =
      declare_parameter<std::string>("obstacle_mesh_topic", "/fr3/lmpc_obstacle_mesh");

  rebuildQpMatrices();
  if (default_table_enabled_) installDefaultTable();

  auto robot_qos = rclcpp::QoS(1).transient_local().reliable();
  robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic, robot_qos,
      std::bind(&LmpcPositionNode::robotDescriptionCallback, this, std::placeholders::_1));
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, rclcpp::SensorDataQoS(),
      std::bind(&LmpcPositionNode::jointStateCallback, this, std::placeholders::_1));
  desired_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      desired_pose_topic, rclcpp::QoS(1).reliable(),
      std::bind(&LmpcPositionNode::desiredPoseCallback, this, std::placeholders::_1));
  mesh_marker_sub_ = create_subscription<visualization_msgs::msg::Marker>(
      obstacle_mesh_topic, rclcpp::QoS(10).reliable(),
      std::bind(&LmpcPositionNode::meshMarkerCallback, this, std::placeholders::_1));
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      twist_topic, rclcpp::QoS(1).best_effort().durability_volatile());
  current_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      current_pose_topic, rclcpp::QoS(1).reliable());

  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency_),
                             std::bind(&LmpcPositionNode::update, this));

  RCLCPP_INFO(get_logger(),
              "LMPC first-order velocity model: %s -> %s at %.1f Hz, horizon=%d, tau=%.3f s, twist_regularization=%.4f, delta_twist_weight=%.4f, watchdog=%.3f s, obstacle meshes=%zu",
              root_link_.c_str(), tip_link_.c_str(), frequency_, horizon_,
              velocity_tracking_time_constant_, regularization_gain_,
              delta_twist_weight_, command_timeout_, meshes_.size());
}

void LmpcPositionNode::robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
  if (kinematics_ready_) return;
  std::string error;
  if (!kinematics_.initialize(msg->data, root_link_, tip_link_, &error)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.c_str());
    return;
  }
  kinematics_ready_ = true;
  RCLCPP_INFO(get_logger(), "Kinematics initialized from /robot_description");
}

void LmpcPositionNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  Vector7d q, dq;
  if (!extractJointState(*msg, joint_names_, q, dq)) return;
  std::lock_guard<std::mutex> lock(mutex_);
  q_ = q;
  dq_ = dq;
  have_joint_state_ = true;
}

void LmpcPositionNode::desiredPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != root_link_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Desired pose frame '%s' differs from controller root '%s'; no TF transform is applied",
                         msg->header.frame_id.c_str(), root_link_.c_str());
  }
  std::lock_guard<std::mutex> lock(mutex_);
  desired_pose_ = poseMsgToIsometry(msg->pose);
  last_pose_command_time_steady_ = std::chrono::steady_clock::now();
  have_desired_pose_ = true;
}

void LmpcPositionNode::meshMarkerCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != root_link_) {
    RCLCPP_WARN(get_logger(),
                "Ignoring obstacle mesh id=%d in frame '%s'; LMPC expects frame '%s' and does not apply TF",
                msg->id, msg->header.frame_id.c_str(), root_link_.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->action == visualization_msgs::msg::Marker::DELETEALL) {
    meshes_.clear();
    RCLCPP_INFO(get_logger(), "Deleted all LMPC obstacle meshes");
    return;
  }
  if (msg->action == visualization_msgs::msg::Marker::DELETE) {
    const std::size_t removed = meshes_.erase(msg->id);
    RCLCPP_INFO(get_logger(), "Deleted LMPC obstacle mesh id=%d%s", msg->id,
                removed ? "" : " (id was not present)");
    return;
  }
  if (msg->type != visualization_msgs::msg::Marker::TRIANGLE_LIST) {
    RCLCPP_WARN(get_logger(), "Ignoring obstacle mesh id=%d: Marker type must be TRIANGLE_LIST", msg->id);
    return;
  }
  if (msg->points.size() < 3 || (msg->points.size() % 3) != 0) {
    RCLCPP_WARN(get_logger(), "Ignoring obstacle mesh id=%d: TRIANGLE_LIST point count must be a non-zero multiple of 3", msg->id);
    return;
  }

  const Eigen::Isometry3d pose = markerPoseToIsometry(msg->pose);
  Eigen::Vector3d scale(msg->scale.x, msg->scale.y, msg->scale.z);
  for (int i = 0; i < 3; ++i) {
    if (std::abs(scale(i)) < 1e-12) scale(i) = 1.0;
  }

  Mesh mesh;
  mesh.id = msg->id;
  mesh.one_sided = false;  // Runtime meshes are ordinary two-sided obstacles.
  mesh.triangles.reserve(msg->points.size() / 3);
  for (std::size_t i = 0; i < msg->points.size(); i += 3) {
    auto toEigen = [&](const geometry_msgs::msg::Point& point) {
      Eigen::Vector3d local(point.x * scale.x(), point.y * scale.y(), point.z * scale.z());
      return pose * local;
    };
    Triangle triangle{toEigen(msg->points[i]), toEigen(msg->points[i + 1]), toEigen(msg->points[i + 2])};
    if ((triangle.b - triangle.a).cross(triangle.c - triangle.a).norm() > 1e-10) {
      mesh.triangles.push_back(triangle);
    }
  }
  if (mesh.triangles.empty()) {
    RCLCPP_WARN(get_logger(), "Ignoring obstacle mesh id=%d: no non-degenerate triangles", msg->id);
    return;
  }
  meshes_[mesh.id] = std::move(mesh);
  RCLCPP_INFO(get_logger(), "Stored LMPC obstacle mesh id=%d with %zu triangles", msg->id,
              meshes_[msg->id].triangles.size());
}

void LmpcPositionNode::installDefaultTable() {
  if (!(default_table_x_min_ < default_table_x_max_) ||
      !(default_table_y_min_ < default_table_y_max_)) {
    throw std::runtime_error("Default table bounds must satisfy min < max");
  }
  Mesh table;
  table.id = default_table_id_;
  table.one_sided = true;
  table.plane_normal = Eigen::Vector3d::UnitZ();
  table.plane_offset = default_table_z_;
  const Eigen::Vector3d p00(default_table_x_min_, default_table_y_min_, default_table_z_);
  const Eigen::Vector3d p10(default_table_x_max_, default_table_y_min_, default_table_z_);
  const Eigen::Vector3d p11(default_table_x_max_, default_table_y_max_, default_table_z_);
  const Eigen::Vector3d p01(default_table_x_min_, default_table_y_max_, default_table_z_);
  table.triangles.push_back(Triangle{p00, p10, p11});
  table.triangles.push_back(Triangle{p00, p11, p01});
  meshes_[table.id] = table;
}

Eigen::Vector3d LmpcPositionNode::triangleNormal(const Triangle& triangle) {
  Eigen::Vector3d n = (triangle.b - triangle.a).cross(triangle.c - triangle.a);
  const double norm = n.norm();
  if (norm < 1e-12) return Eigen::Vector3d::UnitZ();
  return n / norm;
}

Eigen::Vector3d LmpcPositionNode::closestPointOnTriangle(
    const Eigen::Vector3d& p,
    const Triangle& triangle) {
  const Eigen::Vector3d ab = triangle.b - triangle.a;
  const Eigen::Vector3d ac = triangle.c - triangle.a;
  const Eigen::Vector3d ap = p - triangle.a;
  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) return triangle.a;

  const Eigen::Vector3d bp = p - triangle.b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) return triangle.b;

  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const double v = d1 / (d1 - d3);
    return triangle.a + v * ab;
  }

  const Eigen::Vector3d cp = p - triangle.c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) return triangle.c;

  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const double w = d2 / (d2 - d6);
    return triangle.a + w * ac;
  }

  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return triangle.b + w * (triangle.c - triangle.b);
  }

  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom;
  const double w = vc * denom;
  return triangle.a + ab * v + ac * w;
}

std::vector<LmpcPositionNode::ObstacleConstraint> LmpcPositionNode::buildObstacleConstraints(
    const Eigen::Vector3d& tcp_position) {
  std::vector<ObstacleConstraint> constraints;
  if (!obstacle_avoidance_enabled_) return constraints;

  std::vector<Mesh> meshes;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    meshes.reserve(meshes_.size());
    for (const auto& [id, mesh] : meshes_) {
      (void)id;
      meshes.push_back(mesh);
    }
  }

  constraints.reserve(meshes.size());
  for (const auto& mesh : meshes) {
    if (mesh.one_sided) {
      // The built-in table is finite in X/Y but one-sided in Z. Keep the plane
      // constraint active while the TCP is over (or close to) that rectangle.
      const bool over_table =
          tcp_position.x() >= default_table_x_min_ - obstacle_influence_distance_ &&
          tcp_position.x() <= default_table_x_max_ + obstacle_influence_distance_ &&
          tcp_position.y() >= default_table_y_min_ - obstacle_influence_distance_ &&
          tcp_position.y() <= default_table_y_max_ + obstacle_influence_distance_;
      if (!over_table) continue;

      // Signed distance: below the table stays negative and the normal remains
      // +Z. This fixes the old unsigned-distance behavior that could command
      // further downward motion after crossing the table plane.
      const double signed_distance =
          mesh.plane_normal.dot(tcp_position) - mesh.plane_offset;
      if (signed_distance <= obstacle_influence_distance_) {
        constraints.push_back(
            ObstacleConstraint{mesh.id, signed_distance, mesh.plane_normal, true});
      }
      continue;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
    for (const auto& triangle : mesh.triangles) {
      const Eigen::Vector3d closest = closestPointOnTriangle(tcp_position, triangle);
      Eigen::Vector3d delta = tcp_position - closest;
      double distance = delta.norm();
      Eigen::Vector3d normal;
      if (distance > 1e-9) {
        normal = delta / distance;
      } else {
        normal = triangleNormal(triangle);
        distance = 0.0;
      }
      if (distance < best_distance) {
        best_distance = distance;
        best_normal = normal;
      }
    }
    if (best_distance <= obstacle_influence_distance_) {
      constraints.push_back(ObstacleConstraint{mesh.id, best_distance, best_normal, false});
    }
  }

  std::sort(constraints.begin(), constraints.end(),
            [](const ObstacleConstraint& lhs, const ObstacleConstraint& rhs) {
              return lhs.distance < rhs.distance;
            });
  if (static_cast<int>(constraints.size()) > obstacle_max_active_meshes_) {
    constraints.resize(static_cast<std::size_t>(obstacle_max_active_meshes_));
  }
  return constraints;
}

void LmpcPositionNode::rebuildQpMatrices() {
  const double dt = 1.0 / frequency_;
  const double alpha = std::exp(-dt / velocity_tracking_time_constant_);
  const double velocity_input_gain = 1.0 - alpha;
  const double beta = velocity_tracking_time_constant_ * velocity_input_gain;
  const double command_position_gain = dt - beta;
  const Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();

  n_variables_ = 6 * horizon_;  // Desired Cartesian twist sequence u[0..N-1].

  // Be_ maps the desired-twist sequence to predicted pose error.
  // Bv_ maps the desired-twist sequence to predicted actual TCP velocity.
  Be_ = Eigen::MatrixXd::Zero(n_variables_, n_variables_);
  Bv_ = Eigen::MatrixXd::Zero(n_variables_, n_variables_);
  Qbar_ = Eigen::MatrixXd::Zero(n_variables_, n_variables_);
  Vbar_ = Eigen::MatrixXd::Zero(n_variables_, n_variables_);
  Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(n_variables_, n_variables_);

  // Difference operator:
  //   D*U = [u0, u1-u0, u2-u1, ...].
  // During solve(), the first block is referenced to previous_command_.
  difference_matrix_ = Eigen::MatrixXd::Zero(n_variables_, n_variables_);
  delta_weight_matrix_ = Eigen::MatrixXd::Zero(n_variables_, n_variables_);

  Eigen::MatrixXd e_map = Eigen::MatrixXd::Zero(6, n_variables_);
  Eigen::MatrixXd v_map = Eigen::MatrixXd::Zero(6, n_variables_);

  for (int k = 0; k < horizon_; ++k) {
    Eigen::MatrixXd selector = Eigen::MatrixXd::Zero(6, n_variables_);
    selector.block<6, 6>(0, 6 * k) = I6;

    // Exact zero-order-hold discretization of:
    //   v_dot = (u-v)/tau
    //   e_dot = -v
    // over one LMPC sample.
    const Eigen::MatrixXd next_e_map =
        e_map - beta * v_map - command_position_gain * selector;
    const Eigen::MatrixXd next_v_map =
        alpha * v_map + velocity_input_gain * selector;
    e_map = next_e_map;
    v_map = next_v_map;

    Be_.block(6 * k, 0, 6, n_variables_) = e_map;
    Bv_.block(6 * k, 0, 6, n_variables_) = v_map;

    const Vector6d qdiag = (k == horizon_ - 1) ? terminal_weights_ : state_weights_;
    const Vector6d vdiag =
        (k == horizon_ - 1) ? terminal_velocity_weights_ : velocity_weights_;
    Qbar_.block<6, 6>(6 * k, 6 * k) = qdiag.asDiagonal();
    Vbar_.block<6, 6>(6 * k, 6 * k) = vdiag.asDiagonal();
    Rbar.block<6, 6>(6 * k, 6 * k) = input_weights_.asDiagonal();

    difference_matrix_.block<6, 6>(6 * k, 6 * k) = I6;
    if (k > 0) {
      difference_matrix_.block<6, 6>(6 * k, 6 * (k - 1)) = -I6;
    }
    delta_weight_matrix_.block<6, 6>(6 * k, 6 * k) =
        Vector6d::Constant(delta_twist_weight_).asDiagonal();
  }

  H_ = 2.0 * (Be_.transpose() * Qbar_ * Be_ +
              Bv_.transpose() * Vbar_ * Bv_ + Rbar +
              difference_matrix_.transpose() * delta_weight_matrix_ * difference_matrix_);
  H_.diagonal().array() += 1e-9;

  gradient_ = Eigen::VectorXd::Zero(n_variables_);
  lower_ = Eigen::VectorXd::Zero(n_variables_);
  upper_ = Eigen::VectorXd::Zero(n_variables_);

  // Full-horizon desired-twist limits are simple variable bounds now that the
  // decision variable is u rather than acceleration.
  for (int k = 0; k < horizon_; ++k) {
    for (int i = 0; i < 6; ++i) {
      lower_(6 * k + i) = -std::abs(max_twist_(i));
      upper_(6 * k + i) = std::abs(max_twist_(i));
    }
  }

  H_qp_.resize(static_cast<std::size_t>(n_variables_ * n_variables_));
  g_qp_.resize(static_cast<std::size_t>(n_variables_));
  lb_qp_.resize(static_cast<std::size_t>(n_variables_));
  ub_qp_.resize(static_cast<std::size_t>(n_variables_));
  for (int r = 0; r < n_variables_; ++r) {
    for (int c = 0; c < n_variables_; ++c) {
      H_qp_[static_cast<std::size_t>(r * n_variables_ + c)] = H_(r, c);
    }
  }
}

bool LmpcPositionNode::solve(const Vector6d& error,
                             const Vector6d& measured_twist,
                             const Eigen::Vector3d& tcp_position,
                             Vector6d& command) {
  const double dt = 1.0 / frequency_;
  const double alpha = std::exp(-dt / velocity_tracking_time_constant_);
  const double velocity_input_gain = 1.0 - alpha;
  const double beta = velocity_tracking_time_constant_ * velocity_input_gain;

  // Zero-input prediction from the measured state. Input effects are carried
  // separately by Be_ and Bv_. This is the key difference from the previous
  // model: measured velocity decays only according to the identified actuator
  // time constant instead of being assumed directly controllable.
  Eigen::VectorXd error_base(n_variables_);
  Eigen::VectorXd velocity_base(n_variables_);
  Vector6d predicted_error = error;
  Vector6d predicted_velocity = measured_twist;
  for (int k = 0; k < horizon_; ++k) {
    predicted_error -= beta * predicted_velocity;
    predicted_velocity *= alpha;
    error_base.segment<6>(6 * k) = predicted_error;
    velocity_base.segment<6>(6 * k) = predicted_velocity;
  }

  // Delta-command reference. D*U - delta_reference gives
  // [u0-u_prev, u1-u0, ...].
  Eigen::VectorXd delta_reference = Eigen::VectorXd::Zero(n_variables_);
  const Vector6d previous = have_previous_command_ ? previous_command_ : Vector6d::Zero();
  delta_reference.segment<6>(0) = previous;

  gradient_ = 2.0 * (Be_.transpose() * Qbar_ * error_base +
                     Bv_.transpose() * Vbar_ * velocity_base -
                     difference_matrix_.transpose() * delta_weight_matrix_ * delta_reference);

  for (int i = 0; i < n_variables_; ++i) {
    g_qp_[static_cast<std::size_t>(i)] = gradient_(i);
    lb_qp_[static_cast<std::size_t>(i)] = lower_(i);
    ub_qp_[static_cast<std::size_t>(i)] = upper_(i);
  }

  const auto obstacle_constraints = buildObstacleConstraints(tcp_position);
  const int n_obstacle_constraints = static_cast<int>(obstacle_constraints.size());
  const int n_slew_constraints = n_variables_;
  const int n_constraints = n_slew_constraints + n_obstacle_constraints;

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_variables_);
  Eigen::VectorXd lbA = Eigen::VectorXd::Constant(n_constraints, -1e20);
  Eigen::VectorXd ubA = Eigen::VectorXd::Constant(n_constraints, 1e20);

  // Full-horizon command slew/Cartesian acceleration constraints:
  //   |u0-u_prev| <= amax*dt
  //   |uk-u[k-1]| <= amax*dt
  A.block(0, 0, n_slew_constraints, n_variables_) = difference_matrix_;
  for (int k = 0; k < horizon_; ++k) {
    for (int i = 0; i < 6; ++i) {
      const int row = 6 * k + i;
      const double delta = std::abs(max_acceleration_(i)) * dt;
      if (k == 0) {
        lbA(row) = previous(i) - delta;
        ubA(row) = previous(i) + delta;
      } else {
        lbA(row) = -delta;
        ubA(row) = delta;
      }
    }
  }

  // Current obstacle interface remains a one-step velocity barrier, but it is
  // now written against the predicted actual TCP velocity from the first-order
  // plant: v1 = alpha*v_meas + (1-alpha)*u0.
  for (int obstacle_row = 0; obstacle_row < n_obstacle_constraints; ++obstacle_row) {
    const int row = n_slew_constraints + obstacle_row;
    const auto& obstacle = obstacle_constraints[static_cast<std::size_t>(obstacle_row)];
    A.block<1, 3>(row, 0) = velocity_input_gain * obstacle.normal.transpose();
    const double requested_velocity =
        -obstacle_barrier_gain_ * (obstacle.distance - obstacle_safety_distance_);
    const double rhs = requested_velocity -
                       alpha * obstacle.normal.dot(measured_twist.head<3>());

    // Keep the barrier feasible under both the current full-horizon twist box
    // and first-step slew constraints. The position/velocity costs continue the
    // recovery over subsequent MPC updates.
    double max_feasible = 0.0;
    for (int i = 0; i < 3; ++i) {
      const double coeff = A(row, i);
      const double first_lower = std::max(lower_(i), lbA(i));
      const double first_upper = std::min(upper_(i), ubA(i));
      max_feasible += coeff >= 0.0 ? coeff * first_upper : coeff * first_lower;
    }
    lbA(row) = std::min(rhs, max_feasible);
  }

  std::vector<qpOASES::real_t> A_qp(static_cast<std::size_t>(n_constraints * n_variables_));
  std::vector<qpOASES::real_t> lbA_qp(static_cast<std::size_t>(n_constraints));
  std::vector<qpOASES::real_t> ubA_qp(static_cast<std::size_t>(n_constraints));
  for (int r = 0; r < n_constraints; ++r) {
    for (int c = 0; c < n_variables_; ++c) {
      A_qp[static_cast<std::size_t>(r * n_variables_ + c)] = A(r, c);
    }
    lbA_qp[static_cast<std::size_t>(r)] = lbA(r);
    ubA_qp[static_cast<std::size_t>(r)] = ubA(r);
  }

  qpOASES::QProblem qp(n_variables_, n_constraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_NONE;
  qp.setOptions(options);

  int nWSR = std::max(1, max_nwsr_);
  const qpOASES::returnValue status = qp.init(
      H_qp_.data(), g_qp_.data(), A_qp.data(),
      lb_qp_.data(), ub_qp_.data(), lbA_qp.data(), ubA_qp.data(), nWSR);
  if (status != qpOASES::SUCCESSFUL_RETURN) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "LMPC QP solve failed with %d slew + %d obstacle constraints",
                         n_slew_constraints, n_obstacle_constraints);
    return false;
  }

  std::vector<qpOASES::real_t> solution(static_cast<std::size_t>(n_variables_), 0.0);
  if (qp.getPrimalSolution(solution.data()) != qpOASES::SUCCESSFUL_RETURN) return false;

  for (int i = 0; i < 6; ++i) command(i) = solution[static_cast<std::size_t>(i)];
  return command.allFinite();
}

void LmpcPositionNode::update() {
  Vector7d q, dq;
  Eigen::Isometry3d desired;
  bool have_state = false;
  bool have_target = false;
  std::chrono::steady_clock::time_point command_time;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    q = q_;
    dq = dq_;
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

  if (!have_target) return;
  const auto steady_now = std::chrono::steady_clock::now();
  const double command_age =
      std::chrono::duration<double>(steady_now - command_time).count();
  if (command_age > command_timeout_) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_desired_pose_ = false;
    }
    have_previous_command_ = false;
    previous_command_.setZero();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Desired pose watchdog expired (age=%.3f > %.3f s); stopping Cartesian commands",
                         command_age, command_timeout_);
    return;
  }

  const Vector6d error = poseError(current, desired);
  const Vector6d measured_twist = J * dq;
  Vector6d command = Vector6d::Zero();
  if (!solve(error, measured_twist, current.translation(), command)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "LMPC QP solve failed; no twist command published");
    return;
  }
  twist_pub_->publish(makeTwistStamped(command, root_link_, t));
  previous_command_ = command;
  have_previous_command_ = true;
}

}  // namespace franka_cartesian_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<franka_cartesian_control::LmpcPositionNode>());
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("lmpc_position_controller"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
