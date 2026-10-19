#pragma once

#include <array>
#include <chrono>
#include <string>
#include <vector>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace low_level {
class JointPositionController : public controller_interface::ControllerInterface {
 public:
  static constexpr std::size_t N = 7;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  controller_interface::return_type update(const rclcpp::Time&, const rclcpp::Duration&) override;
 private:
  struct PositionCommand {
    std::array<double, N> position{};
    std::chrono::steady_clock::time_point received_at{};
    bool valid{false};
  };
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  std::vector<std::string> joint_names_;
  std::string command_topic_;
  double timeout_{0.1};
  double kp_{1.0};
  double tau_{0.03};
  std::array<double,N> max_velocity_{}, max_acceleration_{}, max_jerk_{};
  std::array<double,N> previous_velocity_{}, previous_acceleration_{};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  realtime_tools::RealtimeBuffer<PositionCommand> buffer_;
};
}  // namespace low_level
