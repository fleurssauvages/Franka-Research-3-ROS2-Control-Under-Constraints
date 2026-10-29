#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_cartesian_control/kinematics.hpp"

namespace franka_cartesian_control {

struct SurfacePlane {
  std::string name;
  Eigen::Vector3d normal{0.0, 0.0, 1.0};
  double offset{0.0};
  double margin{0.0};
  double gain{2.0};
};

inline geometry_msgs::msg::TwistStamped makeCartesianVelocityMessage(
    const Vector6d& velocity, const std::string& frame_id,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.twist.linear.x = velocity(0);
  msg.twist.linear.y = velocity(1);
  msg.twist.linear.z = velocity(2);
  msg.twist.angular.x = velocity(3);
  msg.twist.angular.y = velocity(4);
  msg.twist.angular.z = velocity(5);
  return msg;
}

inline geometry_msgs::msg::WrenchStamped makeCartesianWrenchMessage(
    const Vector6d& wrench, const std::string& frame_id,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.wrench.force.x = wrench(0);
  msg.wrench.force.y = wrench(1);
  msg.wrench.force.z = wrench(2);
  msg.wrench.torque.x = wrench(3);
  msg.wrench.torque.y = wrench(4);
  msg.wrench.torque.z = wrench(5);
  return msg;
}

std::vector<std::string> deriveJointNames(const std::string& robot_type,
                                          const std::string& arm_prefix);
std::string deriveRootLink(const std::string& robot_type, const std::string& arm_prefix);
bool extractJointState(const sensor_msgs::msg::JointState& msg,
                       const std::vector<std::string>& joint_names,
                       Vector7d& q, Vector7d& dq);
bool extractJointEffort(const sensor_msgs::msg::JointState& msg,
                        const std::vector<std::string>& joint_names,
                        Vector7d& effort);
Vector6d twistToVector(const geometry_msgs::msg::TwistStamped& msg);
sensor_msgs::msg::JointState makeVelocityCommand(
    const std::vector<std::string>& joint_names, const Vector7d& qdot,
    const rclcpp::Time& stamp);

Vector7d clampVector(const Vector7d& x, const Vector7d& lower, const Vector7d& upper);
// Uniformly scales the full joint-velocity vector so every joint satisfies its
// bound. Unlike component-wise clipping, this preserves the joint-space
// direction and therefore preserves the Cartesian direction produced by the IK
// as long as the origin lies inside every bound.
Vector7d saturateVectorPreservingDirection(const Vector7d& x, const Vector7d& lower,
                                            const Vector7d& upper, double* scale = nullptr,
                                            bool* origin_feasible = nullptr);
// Applies a uniform acceleration saturation to the change from previous to
// target. All joints share the same scale factor, avoiding per-joint clipping.
Vector7d saturateAccelerationPreservingDirection(const Vector7d& target,
                                                  const Vector7d& previous,
                                                  const Vector7d& max_acceleration,
                                                  double dt, double* scale = nullptr);
void combineJointBounds(const Vector7d& q, const Vector7d& max_velocity,
                        const Vector7d& q_min, const Vector7d& q_max,
                        double joint_limit_margin, double joint_limit_gain,
                        Vector7d& lower, Vector7d& upper);
void applyAccelerationBounds(const Vector7d& previous, const Vector7d& max_acceleration,
                             double dt, Vector7d& lower, Vector7d& upper);
std::vector<SurfacePlane> parseSurfacePlanes(
    const std::vector<std::string>& names, const std::vector<double>& normals,
    const std::vector<double>& offsets, const std::vector<double>& margins,
    const std::vector<double>& gains, bool enabled);

}  // namespace franka_cartesian_control
