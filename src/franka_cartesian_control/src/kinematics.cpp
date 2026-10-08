#include "franka_cartesian_control/kinematics.hpp"

#include <kdl_parser/kdl_parser.hpp>

namespace franka_cartesian_control {

bool Kinematics::initialize(const std::string& urdf, const std::string& root_link,
                            const std::string& tip_link, std::string* error) {
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree)) {
    if (error) *error = "Failed to parse robot_description into a KDL tree";
    return false;
  }
  if (!tree.getChain(root_link, tip_link, chain_)) {
    if (error) *error = "Failed to extract KDL chain from '" + root_link + "' to '" + tip_link + "'";
    return false;
  }
  if (chain_.getNrOfJoints() != kDof) {
    if (error) *error = "Expected a 7-DOF chain but got " + std::to_string(chain_.getNrOfJoints());
    return false;
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
