#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>

namespace low_level {

class GripperControlNode : public rclcpp::Node {
 public:
  using Move = franka_msgs::action::Move;
  using Grasp = franka_msgs::action::Grasp;

  GripperControlNode() : Node("gripper_control_node") {
    command_topic_ = declare_parameter<std::string>("command_topic", "gripper_command");
    move_action_ = declare_parameter<std::string>("move_action", "/franka_gripper/move");
    grasp_action_ = declare_parameter<std::string>("grasp_action", "/franka_gripper/grasp");
    max_width_ = declare_parameter<double>("max_width", 0.08);
    speed_ = declare_parameter<double>("speed", 0.03);
    force_ = declare_parameter<double>("force", 40.0);
    epsilon_inner_ = declare_parameter<double>("epsilon_inner", 0.005);
    epsilon_outer_ = declare_parameter<double>("epsilon_outer", 0.005);
    command_deadband_ = declare_parameter<double>("command_deadband", 0.001);
    close_uses_grasp_ = declare_parameter<bool>("close_uses_grasp", true);

    move_client_ = rclcpp_action::create_client<Move>(this, move_action_);
    grasp_client_ = rclcpp_action::create_client<Grasp>(this, grasp_action_);

    command_sub_ = create_subscription<std_msgs::msg::Float32>(
        command_topic_, rclcpp::QoS(1).reliable(),
        std::bind(&GripperControlNode::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Gripper ratio command: '%s' [0..1]", command_topic_.c_str());
  }

 private:
  void command_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    const double ratio = std::clamp(static_cast<double>(msg->data), 0.0, 1.0);
    if (last_ratio_valid_ && std::abs(ratio - last_ratio_) < command_deadband_) {
      return;
    }

    const bool closing = last_ratio_valid_ && ratio < last_ratio_;
    last_ratio_ = ratio;
    last_ratio_valid_ = true;

    if (closing && close_uses_grasp_) {
      send_grasp(ratio * max_width_);
    } else {
      send_move(ratio * max_width_);
    }
  }

  void send_move(double width) {
    if (!move_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Move action server '%s' is not ready", move_action_.c_str());
      return;
    }

    Move::Goal goal;
    goal.width = width;
    goal.speed = speed_;
    move_client_->async_send_goal(goal);
  }

  void send_grasp(double width) {
    if (!grasp_client_->action_server_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Grasp action server '%s' is not ready", grasp_action_.c_str());
      return;
    }

    Grasp::Goal goal;
    goal.width = width;
    goal.speed = speed_;
    goal.force = force_;
    goal.epsilon.inner = epsilon_inner_;
    goal.epsilon.outer = epsilon_outer_;
    grasp_client_->async_send_goal(goal);
  }

  std::string command_topic_;
  std::string move_action_;
  std::string grasp_action_;
  double max_width_{0.08};
  double speed_{0.03};
  double force_{40.0};
  double epsilon_inner_{0.005};
  double epsilon_outer_{0.005};
  double command_deadband_{0.001};
  bool close_uses_grasp_{true};
  double last_ratio_{0.0};
  bool last_ratio_valid_{false};

  rclcpp_action::Client<Move>::SharedPtr move_client_;
  rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr command_sub_;
};

}  // namespace low_level

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<low_level::GripperControlNode>());
  rclcpp::shutdown();
  return 0;
}
