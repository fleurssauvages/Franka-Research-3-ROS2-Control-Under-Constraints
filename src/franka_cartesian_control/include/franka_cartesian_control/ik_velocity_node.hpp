#pragma once

#include <atomic>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include "franka_cartesian_control/common.hpp"

namespace franka_cartesian_control {

class IkVelocityNode : public rclcpp::Node {
 public:
  IkVelocityNode();

 private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void update();
  void publishZero();

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
  bool have_last_update_time_{false};
  bool ready_published_{false};

  std::string robot_type_;
  std::string arm_prefix_;
  std::string root_link_;
  std::string tip_link_;
  std::vector<std::string> joint_names_;
  Vector7d max_velocity_;
  Vector7d max_acceleration_;
  Vector7d q_min_;
  Vector7d q_max_;
  Vector7d nullspace_target_;
  std::vector<SurfacePlane> surface_planes_;
  double frequency_{200.0};
  double command_timeout_{0.1};
  double pinv_damping_{0.0001};
  double nullspace_gain_{0.0};
  double joint_limit_margin_{0.08};
  double joint_limit_gain_{2.0};
  bool enforce_surfaces_{true};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace franka_cartesian_control
