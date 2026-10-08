#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "franka_cartesian_control/kinematics.hpp"

namespace franka_cartesian_control {

inline Vector6d poseError(const Eigen::Isometry3d& current,
                          const Eigen::Isometry3d& desired) {
  Vector6d error = Vector6d::Zero();
  error.head<3>() = desired.translation() - current.translation();

  Eigen::Matrix3d R_error = desired.linear() * current.linear().transpose();
  Eigen::AngleAxisd aa(R_error);
  double angle = aa.angle();
  constexpr double kPi = 3.14159265358979323846;
  if (angle > kPi) angle -= 2.0 * kPi;
  if (std::abs(angle) > 1e-12) {
    error.tail<3>() = aa.axis() * angle;
  }
  return error;
}

inline Eigen::Isometry3d poseMsgToIsometry(const geometry_msgs::msg::Pose& pose) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() << pose.position.x, pose.position.y, pose.position.z;
  Eigen::Quaterniond q(pose.orientation.w,
                       pose.orientation.x,
                       pose.orientation.y,
                       pose.orientation.z);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  out.linear() = q.toRotationMatrix();
  return out;
}

inline geometry_msgs::msg::PoseStamped makePoseStamped(
    const Eigen::Isometry3d& pose,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.pose.position.x = pose.translation().x();
  msg.pose.position.y = pose.translation().y();
  msg.pose.position.z = pose.translation().z();
  Eigen::Quaterniond q(pose.linear());
  q.normalize();
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  return msg;
}

inline geometry_msgs::msg::TwistStamped makeTwistStamped(
    const Vector6d& twist,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.twist.linear.x = twist(0);
  msg.twist.linear.y = twist(1);
  msg.twist.linear.z = twist(2);
  msg.twist.angular.x = twist(3);
  msg.twist.angular.y = twist(4);
  msg.twist.angular.z = twist(5);
  return msg;
}

inline Vector6d vector6FromStd(const std::vector<double>& values,
                               const char* name) {
  if (values.size() != 6) {
    throw std::runtime_error(std::string(name) + " must contain 6 values");
  }
  Vector6d out;
  for (int i = 0; i < 6; ++i) out(i) = values[static_cast<std::size_t>(i)];
  return out;
}

inline Vector6d clampAbs(const Vector6d& x, const Vector6d& limit) {
  Vector6d out;
  for (int i = 0; i < 6; ++i) {
    const double lim = std::max(0.0, limit(i));
    out(i) = std::clamp(x(i), -lim, lim);
  }
  return out;
}

inline Vector6d saturateAbsPreservingDirection(const Vector6d& x, const Vector6d& limit) {
  double ratio = 1.0;
  for (int i = 0; i < 6; ++i) {
    const double lim = std::max(0.0, limit(i));
    const double magnitude = std::abs(x(i));
    if (magnitude <= lim) continue;
    if (lim <= 0.0) return Vector6d::Zero();
    ratio = std::max(ratio, magnitude / lim);
  }
  return x / ratio;
}

}  // namespace franka_cartesian_control
