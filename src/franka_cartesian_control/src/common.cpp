#include "franka_cartesian_control/common.hpp"

#include <algorithm>

namespace franka_cartesian_control {

static std::string expandedPrefix(const std::string& robot_type, const std::string& arm_prefix) {
  return (arm_prefix.empty() ? std::string{} : arm_prefix + "_") + robot_type;
}

std::vector<std::string> deriveJointNames(const std::string& robot_type,
                                          const std::string& arm_prefix) {
  std::vector<std::string> names;
  names.reserve(kDof);
  const auto prefix = expandedPrefix(robot_type, arm_prefix);
  for (std::size_t i = 1; i <= kDof; ++i) names.push_back(prefix + "_joint" + std::to_string(i));
  return names;
}

std::string deriveRootLink(const std::string& robot_type, const std::string& arm_prefix) {
  return expandedPrefix(robot_type, arm_prefix) + "_link0";
}

bool extractJointState(const sensor_msgs::msg::JointState& msg,
                       const std::vector<std::string>& joint_names,
                       Vector7d& q, Vector7d& dq) {
  if (msg.name.empty() || msg.position.size() != msg.name.size()) return false;
  // Seven joints are small enough that a bounded linear search is cheaper and
  // allocation-free compared with building an unordered_map on every state
  // message. This also reduces allocator/cache jitter on the Franka RT host.
  for (std::size_t j = 0; j < kDof; ++j) {
    std::size_t index = msg.name.size();
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (msg.name[i] == joint_names[j]) {
        index = i;
        break;
      }
    }
    if (index >= msg.position.size()) return false;
    q(static_cast<int>(j)) = msg.position[index];
    dq(static_cast<int>(j)) = (index < msg.velocity.size()) ? msg.velocity[index] : 0.0;
  }
  return q.allFinite() && dq.allFinite();
}

Vector6d twistToVector(const geometry_msgs::msg::TwistStamped& msg) {
  Vector6d x;
  x << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
       msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
  return x;
}

sensor_msgs::msg::JointState makeVelocityCommand(
    const std::vector<std::string>& joint_names, const Vector7d& qdot,
    const rclcpp::Time& stamp) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = stamp;
  msg.name = joint_names;
  msg.velocity.resize(kDof);
  for (std::size_t i = 0; i < kDof; ++i) msg.velocity[i] = qdot(static_cast<int>(i));
  return msg;
}

Vector7d clampVector(const Vector7d& x, const Vector7d& lower, const Vector7d& upper) {
  Vector7d out;
  for (int i = 0; i < 7; ++i) out(i) = std::clamp(x(i), lower(i), upper(i));
  return out;
}

Vector7d saturateVectorPreservingDirection(const Vector7d& x, const Vector7d& lower,
                                            const Vector7d& upper, double* scale,
                                            bool* origin_feasible) {
  double alpha = 1.0;
  bool feasible = true;
  for (int i = 0; i < 7; ++i) {
    if (lower(i) > 0.0 || upper(i) < 0.0) {
      feasible = false;
      continue;
    }
    if (x(i) > upper(i) && x(i) > 0.0) {
      alpha = std::min(alpha, upper(i) / x(i));
    } else if (x(i) < lower(i) && x(i) < 0.0) {
      alpha = std::min(alpha, lower(i) / x(i));
    }
  }
  alpha = std::clamp(alpha, 0.0, 1.0);
  if (scale) *scale = alpha;
  if (origin_feasible) *origin_feasible = feasible;
  if (!feasible) {
    // Outside a safe joint range, a non-zero recovery velocity can be required,
    // which cannot in general be represented by scaling a vector from zero.
    // Fall back to component bounds only for this recovery case.
    return clampVector(x, lower, upper);
  }
  return alpha * x;
}

Vector7d saturateAccelerationPreservingDirection(const Vector7d& target,
                                                  const Vector7d& previous,
                                                  const Vector7d& max_acceleration,
                                                  double dt, double* scale) {
  if (dt <= 0.0) {
    if (scale) *scale = 0.0;
    return previous;
  }
  const Vector7d delta = target - previous;
  double alpha = 1.0;
  for (int i = 0; i < 7; ++i) {
    const double max_delta = std::max(0.0, max_acceleration(i)) * dt;
    const double abs_delta = std::abs(delta(i));
    if (abs_delta > max_delta && abs_delta > 1e-12) {
      alpha = std::min(alpha, max_delta / abs_delta);
    }
  }
  alpha = std::clamp(alpha, 0.0, 1.0);
  if (scale) *scale = alpha;
  return previous + alpha * delta;
}

void combineJointBounds(const Vector7d& q, const Vector7d& max_velocity,
                        const Vector7d& q_min, const Vector7d& q_max,
                        double joint_limit_margin, double joint_limit_gain,
                        Vector7d& lower, Vector7d& upper) {
  for (int i = 0; i < 7; ++i) {
    const double lo_safe = q_min(i) + joint_limit_margin;
    const double hi_safe = q_max(i) - joint_limit_margin;
    lower(i) = std::max(-max_velocity(i), -joint_limit_gain * (q(i) - lo_safe));
    upper(i) = std::min( max_velocity(i),  joint_limit_gain * (hi_safe - q(i)));
    if (lower(i) > upper(i)) {
      const double mid = 0.5 * (lower(i) + upper(i));
      lower(i) = mid;
      upper(i) = mid;
    }
  }
}

void applyAccelerationBounds(const Vector7d& previous, const Vector7d& max_acceleration,
                             double dt, Vector7d& lower, Vector7d& upper) {
  if (dt <= 0.0) return;
  for (int i = 0; i < 7; ++i) {
    const double delta = std::max(0.0, max_acceleration(i)) * dt;
    lower(i) = std::max(lower(i), previous(i) - delta);
    upper(i) = std::min(upper(i), previous(i) + delta);
  }
}

std::vector<SurfacePlane> parseSurfacePlanes(
    const std::vector<std::string>& names, const std::vector<double>& normals,
    const std::vector<double>& offsets, const std::vector<double>& margins,
    const std::vector<double>& gains, bool enabled) {
  std::vector<SurfacePlane> planes;
  if (!enabled) return planes;
  const std::size_t n = offsets.size();
  if (n == 0) return planes;
  if (normals.size() != 3 * n || margins.size() != n || gains.size() != n) {
    throw std::runtime_error("surface plane arrays must have sizes normals=3N, offsets=N, margins=N, gains=N");
  }
  if (!names.empty() && names.size() != n) {
    throw std::runtime_error("surface plane names must be empty or contain N entries");
  }
  for (std::size_t i = 0; i < n; ++i) {
    SurfacePlane p;
    p.name = names.empty() ? ("plane_" + std::to_string(i)) : names[i];
    p.normal << normals[3*i], normals[3*i+1], normals[3*i+2];
    const double norm = p.normal.norm();
    if (norm < 1e-9) throw std::runtime_error("surface plane normal cannot be zero");
    p.normal /= norm;
    p.offset = offsets[i] / norm;
    p.margin = margins[i];
    p.gain = gains[i];
    planes.push_back(p);
  }
  return planes;
}

}  // namespace franka_cartesian_control
