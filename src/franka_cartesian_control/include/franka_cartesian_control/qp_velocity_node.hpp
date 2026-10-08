#pragma once

#include <chrono>
#include <mutex>
#include <memory>

#include <qpOASES.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "franka_cartesian_control/common.hpp"

namespace franka_cartesian_control {

class QpVelocityNode : public rclcpp::Node {
 public:
  QpVelocityNode();

 private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void update();
  bool solveQp(const Matrix67d& J, const Eigen::Isometry3d& pose,
               const Vector6d& twist, double dt, Vector7d& solution,
               bool tracking_enabled);
  bool updateJointRecoveryState(const Vector7d& q);
  bool updateSurfaceRecoveryState(const Eigen::Isometry3d& pose);
  bool anyRecoveryStateActive() const;

  std::mutex mutex_;
  Kinematics kinematics_;
  bool kinematics_ready_{false};
  bool have_joint_state_{false};
  bool have_twist_{false};
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector6d desired_twist_{Vector6d::Zero()};
  Vector7d previous_command_{Vector7d::Zero()};
  std::chrono::steady_clock::time_point last_twist_time_steady_{};
  std::chrono::steady_clock::time_point last_update_time_steady_{};
  std::chrono::steady_clock::time_point last_idle_safety_check_time_steady_{};
  bool have_last_update_time_{false};
  bool ready_published_{false};
  bool have_last_idle_safety_check_time_{false};

  std::string robot_type_;
  std::string arm_prefix_;
  std::string root_link_;
  std::string tip_link_;
  std::vector<std::string> joint_names_;
  Vector7d max_velocity_;
  Vector7d max_acceleration_;
  Vector7d q_min_;
  Vector7d q_max_;
  Vector7d posture_target_;
  Vector6d task_weights_;
  std::vector<SurfacePlane> surface_planes_;
  std::vector<double> surface_repulsion_gains_;
  std::vector<double> surface_repulsion_weights_;
  std::vector<double> surface_max_repulsion_velocities_;
  std::vector<double> surface_release_distances_;
  std::vector<bool> surface_recovery_active_;
  std::array<int, 7> joint_recovery_direction_{};
  Vector7d recovery_command_{Vector7d::Zero()};

  double frequency_{200.0};
  double command_timeout_{0.1};
  double idle_safety_frequency_{50.0};
  double joint_limit_margin_{0.08};
  double joint_limit_gain_{2.0};
  double joint_repulsion_gain_{2.0};
  double joint_repulsion_weight_{0.25};
  double max_joint_repulsion_velocity_{0.35};
  double joint_recovery_release_margin_{0.03};
  double recovery_command_time_constant_{0.12};
  double recovery_release_tolerance_{0.01};
  double regularization_{1e-4};
  double pinv_damping_{0.0001};
  double posture_weight_{0.02};
  double posture_gain_{0.5};
  int max_working_set_recalculations_{100};
  bool enforce_surfaces_{true};
  bool repulsion_when_idle_{true};
  bool recovery_active_{false};
  bool stopping_on_timeout_{false};

  std::unique_ptr<qpOASES::SQProblem> qp_;
  bool qp_initialized_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace franka_cartesian_control
