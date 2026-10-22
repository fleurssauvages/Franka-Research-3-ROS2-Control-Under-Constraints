#pragma once

#include <array>
#include <chrono>
#include <mutex>

#include <franka_msgs/msg/franka_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
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
  // Always solve the standard seven-joint arm up to <robot_type>_link8.
  bool initializeFromRobotDescription(const std::string& urdf, const std::string& root_link,
                                      std::string* error);
  // Use the live flange->EE frame from the Franka robot-state broadcaster.
  void subscribeToFrankaState(rclcpp::Node& node, const std::string& topic,
                              double timeout_seconds);
  bool tcpReady() const;
  bool ready() const { return ready_; }
  const std::string& flangeLink() const { return resolved_flange_link_; }
  // Output Jacobian twist [linear; angular] and pose at the Desk-defined EE.
  bool compute(const Vector7d& q, Matrix67d& jacobian,
               Eigen::Isometry3d& root_T_tip) const;

 private:
  void frankaStateCallback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg);
  bool freshToolTransform(Eigen::Isometry3d& flange_T_ee) const;

  mutable std::mutex tool_mutex_;
  bool have_tool_transform_{false};
  Eigen::Isometry3d flange_T_ee_{Eigen::Isometry3d::Identity()};
  std::chrono::steady_clock::time_point last_tool_update_{};
  double state_timeout_seconds_{0.5};
  rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_sub_;
  bool ready_{false};
  std::string resolved_flange_link_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

Eigen::Matrix<double, 7, 6> dampedPseudoInverse(
    const Matrix67d& J, double damping);

}  // namespace franka_cartesian_control
