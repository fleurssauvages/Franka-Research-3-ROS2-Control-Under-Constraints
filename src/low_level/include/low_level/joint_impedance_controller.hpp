#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include "low_level/residual_torque_compensator.hpp"

namespace low_level {

class JointImpedanceController : public controller_interface::ControllerInterface {
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
  struct ImpedanceCommand {
    std::array<double, kNumJoints> position{};
    std::array<double, kNumJoints> velocity{};
    std::array<double, kNumJoints> effort{};
    std::chrono::steady_clock::time_point received_at{};
    bool has_position{false};
    bool has_velocity{false};
    bool has_effort{false};
    bool valid{false};
  };

  void command_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  bool extract_field(const sensor_msgs::msg::JointState& msg,
                     const std::vector<double>& field,
                     std::array<double, kNumJoints>& output) const;
  std::vector<std::string> derive_joint_names() const;
  bool read_joint_state(std::array<double, kNumJoints>& q,
                        std::array<double, kNumJoints>& dq) const;
  double friction_compensation_torque(std::size_t joint_i, double dq) const;
  void calibration_callback(const std_msgs::msg::Bool::SharedPtr msg);

  struct CalibrationAuthorization {
    bool enabled{false};
    std::chrono::steady_clock::time_point received_at{};
  };

  std::string robot_type_;
  std::string arm_prefix_;
  std::vector<std::string> joint_names_;
  std::string command_topic_;

  double command_timeout_s_{0.01};
  double effort_feedforward_scale_{1.0};
  double delta_tau_max_{1.0};
  bool hold_position_on_timeout_{false};

  std::array<double, kNumJoints> stiffness_{};
  std::array<double, kNumJoints> damping_{};
  std::array<double, kNumJoints> mass_damping_{};
  std::array<double, kNumJoints> max_torque_{};
  std::array<double, kNumJoints> last_tau_command_{};
  std::array<double, kNumJoints> timeout_hold_position_{};
  bool timeout_hold_initialized_{false};
  bool use_mass_damping_{false};

  // Local, explicitly operator-authorized residual learning. Inactive by default.
  bool gravity_error_compensation_enabled_{false};
  double gravity_error_calibration_lease_s_{0.35};
  ResidualTorqueCompensator::Settings residual_settings_{};
  std::unique_ptr<ResidualTorqueCompensator> residual_compensator_;
  realtime_tools::RealtimeBuffer<CalibrationAuthorization> calibration_buffer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibration_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr residual_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      residual_realtime_pub_;
  double residual_publish_s_{0.0};

  // Experimental positive breakaway assistance; disabled by default.
  bool breakaway_enabled_{false};
  double breakaway_velocity_epsilon_{0.004};
  double breakaway_deadband_{0.6};
  double breakaway_gain_{0.10};
  double breakaway_max_torque_{0.08};
  double breakaway_slew_rate_{0.10};
  std::array<double, kNumJoints> breakaway_applied_{};

  // Optional calibrated friction feedforward. Calibration is selected and
  // freshness-checked by the launch file; the RT update loop only uses these
  // fixed-size arrays and performs no filesystem or parameter access.
  bool friction_compensation_requested_{true};
  bool friction_calibration_valid_{false};
  bool friction_compensation_active_{false};
  double friction_compensation_scale_{1.0};
  double friction_smoothing_velocity_{0.01};
  bool friction_stribeck_enabled_{true};
  std::array<double, kNumJoints> friction_stribeck_velocity_{};
  std::array<double, kNumJoints> friction_static_{};
  std::array<double, kNumJoints> friction_max_compensation_torque_{};
  double friction_validated_velocity_min_{0.02}; // Legacy scalar compatibility
  std::array<double, kNumJoints> friction_validated_velocity_min_per_joint_{};
  std::array<double, kNumJoints> friction_validated_velocity_max_{};
  double friction_calibration_age_hours_{-1.0};
  std::string friction_calibration_source_;
  std::array<bool, kNumJoints> friction_joint_enable_{};
  std::array<double, kNumJoints> friction_coulomb_{};
  std::array<double, kNumJoints> friction_viscous_{};

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  static constexpr const char* kRobotStateInterfaceName = "robot_state";
  static constexpr const char* kRobotModelInterfaceName = "robot_model";

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  realtime_tools::RealtimeBuffer<ImpedanceCommand> command_buffer_;
};

}  // namespace low_level
