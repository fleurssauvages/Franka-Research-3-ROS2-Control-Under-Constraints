#include "franka_cartesian_control/kinematics.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include <cmath>
#include <stdexcept>

namespace franka_cartesian_control {

bool Kinematics::initializeFromRobotDescription(const std::string& urdf,
                                                 const std::string& root_link,
                                                 std::string* error) {
  ready_ = false;
  resolved_flange_link_.clear();
  jac_solver_.reset();
  fk_solver_.reset();
  chain_ = KDL::Chain();

  // The URDF is used only for the base->link8 arm kinematics, never to
  // determine the tool. The actual tool frame comes from Franka Desk.
  const std::string suffix = "_link0";
  if (root_link.size() <= suffix.size() ||
      root_link.compare(root_link.size() - suffix.size(), suffix.size(), suffix) != 0) {
    if (error) *error = "Expected root link ending in _link0: " + root_link;
    return false;
  }
  const std::string flange_link = root_link.substr(0, root_link.size() - suffix.size()) + "_link8";
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree) ||
      !tree.getChain(root_link, flange_link, chain_)) {
    if (error) *error = "Cannot construct KDL chain " + root_link + " -> " + flange_link;
    return false;
  }
  if (chain_.getNrOfJoints() != kDof) {
    if (error) *error = "KDL chain to " + flange_link + " must have exactly 7 joints";
    return false;
  }
  resolved_flange_link_ = flange_link;
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ready_ = true;
  return true;
}

void Kinematics::subscribeToFrankaState(rclcpp::Node& node,
                                       const std::string& topic, double timeout_seconds) {
  if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0) {
    throw std::invalid_argument("franka_state_timeout must be > 0");
  }
  state_timeout_seconds_ = timeout_seconds;
  const auto logger = node.get_logger();
  franka_state_sub_ = node.create_subscription<franka_msgs::msg::FrankaRobotState>(
      topic, rclcpp::SensorDataQoS(),
      [this, logger](franka_msgs::msg::FrankaRobotState::SharedPtr msg) {
        frankaStateCallback(msg);
        Eigen::Isometry3d flange_T_ee;
        if (freshToolTransform(flange_T_ee)) {
          RCLCPP_INFO_ONCE(
              logger,
              "FRANKA_DESK_EE: received live F_T_EE: translation [%.4f, %.4f, %.4f] m; "
              "using KDL link8 + Desk frame for all Cartesian kinematics",
              flange_T_ee.translation().x(), flange_T_ee.translation().y(),
              flange_T_ee.translation().z());
        }
      });
}

void Kinematics::frankaStateCallback(
    const franka_msgs::msg::FrankaRobotState::SharedPtr msg) {
  const auto& pose = msg->f_t_ee.pose;  // flange_T_EE, from robot/Desk
  const Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x,
                              pose.orientation.y, pose.orientation.z);
  const double norm = rotation.norm();
  if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 0.05) {
    // Never substitute the flange or identity for an invalid tool transform.
    std::lock_guard<std::mutex> lock(tool_mutex_);
    have_tool_transform_ = false;
    return;
  }
  rotation.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() = translation;
  {
    std::lock_guard<std::mutex> lock(tool_mutex_);
    flange_T_ee_ = transform;
    last_tool_update_ = std::chrono::steady_clock::now();
    have_tool_transform_ = true;
  }
}

bool Kinematics::freshToolTransform(Eigen::Isometry3d& flange_T_ee) const {
  std::lock_guard<std::mutex> lock(tool_mutex_);
  if (!have_tool_transform_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_tool_update_).count()
          > state_timeout_seconds_) {
    return false;
  }
  flange_T_ee = flange_T_ee_;
  return true;
}

bool Kinematics::tcpReady() const {
  Eigen::Isometry3d transform;
  return ready_ && freshToolTransform(transform);
}

bool Kinematics::compute(const Vector7d& q, Matrix67d& jacobian,
                         Eigen::Isometry3d& root_T_tip) const {
  if (!ready_ || !q.allFinite()) return false;
  Eigen::Isometry3d flange_T_ee;
  if (!freshToolTransform(flange_T_ee)) return false;
  KDL::JntArray q_kdl(kDof);
  for (std::size_t i = 0; i < kDof; ++i) q_kdl(i) = q(static_cast<int>(i));

  KDL::Jacobian J_kdl(kDof);
  KDL::Frame frame;
  if (jac_solver_->JntToJac(q_kdl, J_kdl) < 0 || fk_solver_->JntToCart(q_kdl, frame) < 0) {
    return false;
  }

  Eigen::Isometry3d root_T_flange = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) root_T_flange.linear()(r, c) = frame.M(r, c);
  }
  root_T_flange.translation() << frame.p.x(), frame.p.y(), frame.p.z();
  root_T_tip = root_T_flange * flange_T_ee;

  // KDL Jacobian is based at link8, expressed in the root frame. Shift the
  // linear velocity to the Desk-defined end effector; angular part is unchanged.
  // v_EE = v_link8 + omega x (R_root_link8 * p_link8_EE).
  jacobian = J_kdl.data;
  const Eigen::Vector3d r = root_T_flange.linear() * flange_T_ee.translation();
  for (int i = 0; i < static_cast<int>(kDof); ++i) {
    jacobian.col(i).head<3>() += jacobian.col(i).tail<3>().cross(r);
  }
  return root_T_tip.matrix().allFinite() && jacobian.allFinite();
}

Eigen::Matrix<double, 7, 6> dampedPseudoInverse(const Matrix67d& J, double damping) {
  Eigen::Matrix<double, 6, 6> A = J * J.transpose();
  A.diagonal().array() += damping * damping;
  return J.transpose() * A.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());
}

}  // namespace franka_cartesian_control
