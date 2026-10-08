#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

template <std::size_t N>
bool copy_thresholds(
    const std::vector<double>& values,
    std::array<double, N>& target,
    const std::string& name,
    const rclcpp::Logger& logger) {
  if (values.size() != N) {
    RCLCPP_ERROR(logger, "%s must contain exactly %zu values, got %zu", name.c_str(), N,
                 values.size());
    return false;
  }
  std::copy(values.begin(), values.end(), target.begin());
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("collision_behavior_configurator");

  const auto service_name = node->declare_parameter<std::string>(
      "service_name", "/service_server/set_full_collision_behavior");
  const double service_timeout_s = node->declare_parameter<double>("service_timeout", 15.0);

  const auto default_joint = std::vector<double>{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0};
  const auto default_cartesian = std::vector<double>{20.0, 20.0, 20.0, 25.0, 25.0, 25.0};

  const auto lower_tau_acc = node->declare_parameter<std::vector<double>>(
      "lower_torque_thresholds_acceleration", default_joint);
  const auto upper_tau_acc = node->declare_parameter<std::vector<double>>(
      "upper_torque_thresholds_acceleration", default_joint);
  const auto lower_tau_nom = node->declare_parameter<std::vector<double>>(
      "lower_torque_thresholds_nominal", default_joint);
  const auto upper_tau_nom = node->declare_parameter<std::vector<double>>(
      "upper_torque_thresholds_nominal", default_joint);

  const auto lower_force_acc = node->declare_parameter<std::vector<double>>(
      "lower_force_thresholds_acceleration", default_cartesian);
  const auto upper_force_acc = node->declare_parameter<std::vector<double>>(
      "upper_force_thresholds_acceleration", default_cartesian);
  const auto lower_force_nom = node->declare_parameter<std::vector<double>>(
      "lower_force_thresholds_nominal", default_cartesian);
  const auto upper_force_nom = node->declare_parameter<std::vector<double>>(
      "upper_force_thresholds_nominal", default_cartesian);

  auto request = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();
  bool valid = true;
  valid &= copy_thresholds(lower_tau_acc, request->lower_torque_thresholds_acceleration,
                           "lower_torque_thresholds_acceleration", node->get_logger());
  valid &= copy_thresholds(upper_tau_acc, request->upper_torque_thresholds_acceleration,
                           "upper_torque_thresholds_acceleration", node->get_logger());
  valid &= copy_thresholds(lower_tau_nom, request->lower_torque_thresholds_nominal,
                           "lower_torque_thresholds_nominal", node->get_logger());
  valid &= copy_thresholds(upper_tau_nom, request->upper_torque_thresholds_nominal,
                           "upper_torque_thresholds_nominal", node->get_logger());
  valid &= copy_thresholds(lower_force_acc, request->lower_force_thresholds_acceleration,
                           "lower_force_thresholds_acceleration", node->get_logger());
  valid &= copy_thresholds(upper_force_acc, request->upper_force_thresholds_acceleration,
                           "upper_force_thresholds_acceleration", node->get_logger());
  valid &= copy_thresholds(lower_force_nom, request->lower_force_thresholds_nominal,
                           "lower_force_thresholds_nominal", node->get_logger());
  valid &= copy_thresholds(upper_force_nom, request->upper_force_thresholds_nominal,
                           "upper_force_thresholds_nominal", node->get_logger());

  if (!valid) {
    rclcpp::shutdown();
    return 2;
  }

  auto client = node->create_client<franka_msgs::srv::SetFullCollisionBehavior>(service_name);
  RCLCPP_INFO(node->get_logger(), "Waiting for collision-behavior service '%s'...",
              service_name.c_str());

  if (!client->wait_for_service(std::chrono::duration<double>(service_timeout_s))) {
    RCLCPP_ERROR(node->get_logger(), "Collision-behavior service did not become available within %.1f s",
                 service_timeout_s);
    rclcpp::shutdown();
    return 3;
  }

  auto future = client->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(
      node, future, std::chrono::duration<double>(service_timeout_s));

  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Timed out while setting collision behavior");
    rclcpp::shutdown();
    return 4;
  }

  const auto response = future.get();
  if (!response->success) {
    RCLCPP_ERROR(node->get_logger(), "Franka rejected collision behavior: %s",
                 response->error.c_str());
    rclcpp::shutdown();
    return 5;
  }

  RCLCPP_INFO(node->get_logger(), "Collision behavior configured successfully");
  rclcpp::shutdown();
  return 0;
}
