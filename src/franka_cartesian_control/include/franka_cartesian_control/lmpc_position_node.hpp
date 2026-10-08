#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <qpOASES.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "franka_cartesian_control/common.hpp"
#include "franka_cartesian_control/kinematics.hpp"

namespace franka_cartesian_control {

class LmpcPositionNode : public rclcpp::Node {
 public:
  LmpcPositionNode();

 private:
  struct Triangle {
    Eigen::Vector3d a{Eigen::Vector3d::Zero()};
    Eigen::Vector3d b{Eigen::Vector3d::Zero()};
    Eigen::Vector3d c{Eigen::Vector3d::Zero()};
  };

  struct Mesh {
    int id{0};
    std::vector<Triangle> triangles;
    // The built-in table is a one-sided obstacle: +normal is always the allowed
    // side. Runtime TRIANGLE_LIST meshes remain ordinary two-sided obstacles.
    bool one_sided{false};
    Eigen::Vector3d plane_normal{Eigen::Vector3d::UnitZ()};
    double plane_offset{0.0};
  };

  struct ObstacleConstraint {
    int mesh_id{0};
    double distance{0.0};
    Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
    bool one_sided{false};
  };

  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void desiredPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void meshMarkerCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
  void update();
  void rebuildQpMatrices();
  bool solve(const Vector6d& error,
             const Vector6d& measured_twist,
             const Eigen::Vector3d& tcp_position,
             Vector6d& command);

  void installDefaultTable();
  std::vector<ObstacleConstraint> buildObstacleConstraints(
      const Eigen::Vector3d& tcp_position);
  static Eigen::Vector3d closestPointOnTriangle(
      const Eigen::Vector3d& p,
      const Triangle& triangle);
  static Eigen::Vector3d triangleNormal(const Triangle& triangle);

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
  int horizon_{25};
  int max_nwsr_{100};

  // Cartesian plant model state x = [pose_error, measured_tcp_velocity].
  // The QP input is desired Cartesian twist sent to the downstream IK/QP.
  // Actual TCP velocity follows that command through a first-order model:
  //   v[k+1] = alpha*v[k] + (1-alpha)*u[k]
  // with alpha = exp(-dt / velocity_tracking_time_constant).
  Vector6d state_weights_{Vector6d::Ones()};
  Vector6d terminal_weights_{(Vector6d() << 30.0, 30.0, 30.0, 20.0, 20.0, 20.0).finished()};
  Vector6d velocity_weights_{(Vector6d() << 1.0, 1.0, 1.0, 0.50, 0.50, 0.50).finished()};
  Vector6d terminal_velocity_weights_{(Vector6d() << 50.0, 50.0, 50.0, 20.0, 20.0, 20.0).finished()};
  Vector6d input_weights_{Vector6d::Constant(0.01)};
  double regularization_gain_{0.01};
  double delta_twist_weight_{0.5};
  double velocity_tracking_time_constant_{0.08};
  Vector6d max_twist_{(Vector6d() << 0.50, 0.50, 0.50, 0.20, 0.20, 0.20).finished()};
  // Maximum slew rate of the desired Cartesian twist command.
  Vector6d max_acceleration_{(Vector6d() << 2.0, 2.0, 2.0, 4.0, 4.0, 4.0).finished()};

  bool obstacle_avoidance_enabled_{true};
  double obstacle_safety_distance_{0.05};
  double obstacle_influence_distance_{0.15};
  double obstacle_barrier_gain_{3.0};
  int obstacle_max_active_meshes_{16};
  bool default_table_enabled_{true};
  int default_table_id_{0};
  double default_table_z_{0.0};
  double default_table_x_min_{-1.0};
  double default_table_x_max_{1.0};
  double default_table_y_min_{-1.0};
  double default_table_y_max_{1.0};
  std::map<int, Mesh> meshes_;

  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};
  Eigen::Isometry3d desired_pose_{Eigen::Isometry3d::Identity()};
  std::chrono::steady_clock::time_point last_pose_command_time_steady_{};
  Vector6d previous_command_{Vector6d::Zero()};
  bool have_previous_command_{false};

  int n_variables_{0};
  Eigen::MatrixXd Be_;
  Eigen::MatrixXd Bv_;
  Eigen::MatrixXd Qbar_;
  Eigen::MatrixXd Vbar_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd difference_matrix_;
  Eigen::MatrixXd delta_weight_matrix_;
  Eigen::VectorXd gradient_;
  Eigen::VectorXd lower_;
  Eigen::VectorXd upper_;
  std::vector<qpOASES::real_t> H_qp_;
  std::vector<qpOASES::real_t> g_qp_;
  std::vector<qpOASES::real_t> lb_qp_;
  std::vector<qpOASES::real_t> ub_qp_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr mesh_marker_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace franka_cartesian_control
