#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "franka_cartesian_control/common.hpp"

namespace franka_cartesian_control {

// Cartesian wrench -> tau = J(q)^T * wrench, sharing KDL kinematics with the
// IK velocity controller. Optional twist mode uses damped differential IK to
// turn a commanded TCP velocity into a bounded Cartesian feedback wrench.
class IkForceNode : public rclcpp::Node {
 public:
  IkForceNode();

 private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void update();
  bool frameIsRoot(const std::string& frame) const;

  std::mutex mutex_;
  Kinematics kinematics_;
  bool kinematics_ready_{false};
  bool have_joint_state_{false};
  bool have_command_{false};
  bool ready_published_{false};
  bool twist_mode_{false};
  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Vector6d desired_cartesian_{Vector6d::Zero()};
  std::chrono::steady_clock::time_point last_state_time_{};
  std::chrono::steady_clock::time_point last_command_time_{};

  std::string root_link_;
  std::string tip_link_;
  std::vector<std::string> joint_names_;
  Vector7d max_velocity_{Vector7d::Zero()};
  Vector7d max_joint_torque_{Vector7d::Zero()};
  Vector7d q_min_{Vector7d::Zero()};
  Vector7d q_max_{Vector7d::Zero()};
  Vector6d velocity_to_wrench_gains_{Vector6d::Zero()};
  double frequency_{200.0};
  double command_timeout_{0.1};
  double joint_state_timeout_{0.2};
  double pinv_damping_{0.02};
  double joint_limit_margin_{0.08};
  double joint_limit_gain_{2.0};
  double max_force_{15.0};
  double max_moment_{2.0};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace franka_cartesian_control
