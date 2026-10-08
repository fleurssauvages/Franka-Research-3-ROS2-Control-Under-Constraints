#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "franka_cartesian_control/common.hpp"
#include "franka_cartesian_control/kinematics.hpp"

namespace franka_cartesian_control {

class PidPositionNode : public rclcpp::Node {
 public:
  PidPositionNode();

 private:
  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void update();

  std::mutex mutex_;
  Kinematics kinematics_;
  bool kinematics_ready_{false};
  bool have_joint_state_{false};
  bool have_desired_pose_{false};

  std::string robot_type_;
  std::string arm_prefix_;
  std::string root_link_;
  std::string tip_link_;
  std::vector<std::string> joint_names_;

  double frequency_{50.0};
  double command_timeout_{0.50};
  bool reset_integral_on_new_target_{true};

  Vector6d kp_{Vector6d::Constant(5.0)};
  Vector6d ki_{Vector6d::Zero()};
  Vector6d kd_{Vector6d::Zero()};
  Vector6d integral_limit_{Vector6d::Constant(0.2)};
  Vector6d max_twist_{(Vector6d() << 0.50, 0.50, 0.50, 0.20, 0.20, 0.20).finished()};

  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Eigen::Isometry3d desired_pose_{Eigen::Isometry3d::Identity()};
  Vector6d integral_error_{Vector6d::Zero()};
  Vector6d previous_error_{Vector6d::Zero()};
  bool have_previous_error_{false};

  std::chrono::steady_clock::time_point last_pose_command_time_steady_{};
  std::chrono::steady_clock::time_point last_update_time_steady_{};
  bool have_last_update_time_{false};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace franka_cartesian_control
