#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace low_level {

class DynamicsObserverController : public controller_interface::ControllerInterface {
 public:
  static constexpr std::size_t kNumJoints = 7;
  static constexpr std::size_t kObservationSize = 112;
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
  std::vector<std::string> derive_joint_names() const;
  bool read_joint_state(std::array<double, kNumJoints>& q,
                        std::array<double, kNumJoints>& dq,
                        std::array<double, kNumJoints>& tau) const;

  std::string robot_type_;
  std::string arm_prefix_;
  std::vector<std::string> joint_names_;
  std::string observation_topic_;

  double publish_rate_{200.0};
  double acceleration_filter_tau_{0.03};
  double publish_accumulator_{0.0};
  bool have_previous_velocity_{false};
  std::array<double, kNumJoints> previous_velocity_{};
  std::array<double, kNumJoints> filtered_acceleration_{};

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  static constexpr const char* kRobotStateInterfaceName = "robot_state";
  static constexpr const char* kRobotModelInterfaceName = "robot_model";

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      realtime_publisher_;
};

}  // namespace low_level
