#include "franka_cartesian_control/kinematics.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <cctype>

namespace franka_cartesian_control {

namespace {

bool endsWith(const std::string& value, const std::string& suffix) {
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}  // namespace

// v7.1: preserve the original C++ symbol for any callers that were compiled
// against v6 headers. Both entry points use the v7 URDF-aware tip resolver.
bool Kinematics::initialize(const std::string& urdf, const std::string& root_link,
                            const std::string& tip_link, std::string* error) {
  return initializeFromRobotDescriptionV7(urdf, root_link, tip_link, error);
}

bool Kinematics::initializeFromRobotDescriptionV7(const std::string& urdf, const std::string& root_link,
                            const std::string& requested_tip_link, std::string* error) {
  ready_ = false;
  resolved_tip_link_.clear();
  jac_solver_.reset();
  fk_solver_.reset();
  chain_ = KDL::Chain();

  // Both compatibility and v7 entry points use this statically linked
  // implementation, so "auto" is never interpreted by the old resolver.
  // `auto` is a launch-safe sentinel. The empty string remains a backward-
  // compatible alias, but users no longer need to pass an empty ROS parameter.
  std::string requested = requested_tip_link;
  const auto first = requested.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    requested.clear();
  } else {
    requested = requested.substr(first, requested.find_last_not_of(" \t\r\n") - first + 1);
  }
  std::string mode = requested;
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (mode == "auto") requested.clear();
  // Explicit guard against ever resolving the "auto" keyword as a URDF link.
  // Only names that appeared in /robot_description may be passed to getChain.
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree)) {
    if (error) *error = "Failed to parse robot_description into a KDL tree";
    return false;
  }

  const auto valid_arm_chain = [&](const std::string& candidate, KDL::Chain& chain) {
    return !candidate.empty() && candidate != root_link &&
           tree.getChain(root_link, candidate, chain) &&
           chain.getNrOfJoints() == kDof;
  };

  if (!requested.empty()) {
    // An explicitly selected TCP is authoritative; an invalid selection is a
    // configuration error, never an invitation to silently pick another link.
    if (!tree.getChain(root_link, requested, chain_)) {
      if (error) {
        *error = "[FR3_KINEMATICS_V7] Requested tip_link '" + requested +
                 "' is not reachable from '" + root_link +
                 "' in /robot_description; check the installed configuration";
      }
      return false;
    }
    if (chain_.getNrOfJoints() != kDof) {
      if (error) {
        *error = "[FR3_KINEMATICS_V7] Requested tip_link '" + requested + "' has " +
                 std::to_string(chain_.getNrOfJoints()) +
                 " movable joints from '" + root_link + "', expected 7";
      }
      return false;
    }
    resolved_tip_link_ = requested;
  } else {
    // Resolve ONLY against the URDF actually published by Franka bringup.
    // The gripper flag changes that description; it is not a separate source
    // of kinematic truth. Prefer a named TCP/EE to a bare flange when present.
    std::string prefix = root_link;
    if (endsWith(prefix, "_link0")) prefix.resize(prefix.size() - 6);
    const std::vector<std::string> preferred = {
        prefix + "_hand_tcp", prefix + "_tcp", prefix + "_ee",
        prefix + "_tool0", prefix + "_link8", prefix + "_flange"};
    for (const auto& candidate : preferred) {
      KDL::Chain candidate_chain;
      if (valid_arm_chain(candidate, candidate_chain)) {
        chain_ = candidate_chain;
        resolved_tip_link_ = candidate;
        break;
      }
    }

    if (resolved_tip_link_.empty()) {
      // Generic URDFs may not use Franka link names. Auto-select only when
      // there is precisely one 7-actuated-joint descendant. Otherwise fail
      // clearly rather than using an arbitrary link as the collision TCP.
      std::vector<std::string> matches;
      KDL::Chain unique_chain;
      for (const auto& entry : tree.getSegments()) {
        KDL::Chain candidate_chain;
        if (valid_arm_chain(entry.first, candidate_chain)) {
          matches.push_back(entry.first);
          if (matches.size() == 1) unique_chain = candidate_chain;
        }
      }
      if (matches.size() == 1) {
        chain_ = unique_chain;
        resolved_tip_link_ = matches.front();
      } else {
        if (error) {
          std::string available;
          for (const auto& match : matches) {
            if (!available.empty()) available += ", ";
            available += match;
          }
          *error = "[FR3_KINEMATICS_V7] Cannot uniquely resolve a 7-DOF Cartesian tip from '" +
                   root_link + "' in /robot_description (" +
                   std::to_string(matches.size()) +
                   " possible links: " + available +
                   "). Set tip_link explicitly to the robot's TCP.";
        }
        return false;
      }
    }
  }

  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ready_ = true;
  return true;
}

bool Kinematics::compute(const Vector7d& q, Matrix67d& jacobian,
                         Eigen::Isometry3d& root_T_tip) const {
  if (!ready_) return false;
  KDL::JntArray q_kdl(kDof);
  for (std::size_t i = 0; i < kDof; ++i) q_kdl(i) = q(static_cast<int>(i));

  KDL::Jacobian J_kdl(kDof);
  KDL::Frame frame;
  if (jac_solver_->JntToJac(q_kdl, J_kdl) < 0 || fk_solver_->JntToCart(q_kdl, frame) < 0) {
    return false;
  }
  jacobian = J_kdl.data;

  root_T_tip.setIdentity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) root_T_tip.linear()(r, c) = frame.M(r, c);
  }
  root_T_tip.translation() << frame.p.x(), frame.p.y(), frame.p.z();
  return true;
}

Eigen::Matrix<double, 7, 6> dampedPseudoInverse(const Matrix67d& J, double damping) {
  Eigen::Matrix<double, 6, 6> A = J * J.transpose();
  A.diagonal().array() += damping * damping;
  return J.transpose() * A.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
}

}  // namespace franka_cartesian_control
