#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

namespace low_level {

class JointVelocityController : public controller_interface::ControllerInterface {
 public:
  static constexpr std::size_t kNumJoints = 7;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct VelocityCommand {
    std::array<double, kNumJoints> velocity{};
    std::chrono::steady_clock::time_point received_at{};
    bool valid{false};
  };

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool extract_velocity(const sensor_msgs::msg::JointState& msg,
                        std::array<double, kNumJoints>& velocity) const;
  std::vector<std::string> derive_joint_names() const;

  std::string robot_type_;
  std::string arm_prefix_;
  std::vector<std::string> joint_names_;
  std::string command_topic_;
  double command_timeout_s_{0.1};
  double velocity_scale_{1.0};
  double tracking_time_constant_s_{0.03};
  std::array<double, kNumJoints> max_velocity_{};
  std::array<double, kNumJoints> max_acceleration_{};
  std::array<double, kNumJoints> max_jerk_{};
  std::array<double, kNumJoints> last_velocity_command_{};
  std::array<double, kNumJoints> last_acceleration_command_{};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  realtime_tools::RealtimeBuffer<VelocityCommand> command_buffer_;
};

}  // namespace low_level
