#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace franka_cartesian_control {

constexpr std::size_t kDof = 7;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;

class Kinematics {
 public:
  // Compatibility entry point for controllers built against the previous API.
  // It uses the same URDF-aware resolver as initializeFromRobotDescriptionV7().
  bool initialize(const std::string& urdf, const std::string& root_link,
                  const std::string& tip_link, std::string* error);
  bool initializeFromRobotDescriptionV7(const std::string& urdf, const std::string& root_link,
                  const std::string& tip_link, std::string* error);
  bool ready() const { return ready_; }
  const std::string& tipLink() const { return resolved_tip_link_; }
  bool compute(const Vector7d& q, Matrix67d& jacobian,
               Eigen::Isometry3d& root_T_tip) const;

 private:
  bool ready_{false};
  std::string resolved_tip_link_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

Eigen::Matrix<double, 7, 6> dampedPseudoInverse(
    const Matrix67d& J, double damping);

}  // namespace franka_cartesian_control
