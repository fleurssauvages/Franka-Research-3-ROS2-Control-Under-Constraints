#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace low_level {

// Experimental, bounded *local* static torque-bias calibration.
// The operator must positively authorize calibration while the arm is unloaded.
// The measured residual is NOT an external-force detector, nor an identifiable
// gravity error during arbitrary physical contact or closed-loop torque output.
class ResidualTorqueCompensator {
 public:
  static constexpr std::size_t kJoints = 7;
  using Joints = std::array<double, kJoints>;

  struct Settings {
    double command_velocity_epsilon{0.002};
    double measured_velocity_epsilon{0.003};
    double stationary_dwell_s{0.5};
    double minimum_sample_s{1.0};
    double filter_tau_s{2.0};
    double sampling_command_torque_epsilon{0.03};
    double sample_deviation_limit{0.10};
    double output_slew_rate{0.10}; // Nm/s
    double pose_radius{0.25}; // rad, maximum joint-wise displacement
    double pose_fade_width{0.25}; // rad
    Joints torque_limits{{0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15}};
  };

  explicit ResidualTorqueCompensator(Settings settings) : settings_(settings) { reset(); }

  void reset() {
    estimate_.fill(0.0);
    applied_.fill(0.0);
    anchor_.fill(0.0);
    filtered_sample_.fill(0.0);
    sample_anchor_.fill(0.0);
    valid_ = false;
    was_authorized_ = false;
    collecting_ = false;
    stationary_s_ = 0.0;
    sample_s_ = 0.0;
  }

  // The caller must supply the *previous cycle's actually sent* output torque.
  // This is used to reject calibration while the torque loop is driving joints.
  // Explicit authorization is required on EVERY sample (prefer a short lease).
  const Joints& update(double dt, bool command_fresh, bool velocity_field_present,
                       const Joints& commanded_velocity, const Joints& q,
                       const Joints& dq, const Joints& measured_effort,
                       const Joints& model_gravity, const Joints& last_command,
                       bool authorized) {
    if (!std::isfinite(dt) || dt <= 0.0) return applied_;

    bool stationary = command_fresh && velocity_field_present;
    for (std::size_t i = 0; i < kJoints; ++i) {
      stationary = stationary && std::isfinite(commanded_velocity[i]) &&
          std::isfinite(q[i]) && std::isfinite(dq[i]) &&
          std::abs(commanded_velocity[i]) <= settings_.command_velocity_epsilon &&
          std::abs(dq[i]) <= settings_.measured_velocity_epsilon;
    }

    // Disable learning as soon as the lease is released / expires. A completed
    // calibration is only committed on release; sampling NEVER applies torque.
    const bool calibration_mode = authorized && command_fresh;
    if (was_authorized_ && !calibration_mode) {
      if (command_fresh && stationary && collecting_ &&
          sample_s_ >= settings_.minimum_sample_s) {
        estimate_ = filtered_sample_;
        anchor_ = sample_anchor_;
        valid_ = true;
      }
      collecting_ = false;
      sample_s_ = 0.0;
    }
    was_authorized_ = calibration_mode;

    if (!stationary || !calibration_mode) {
      stationary_s_ = 0.0;
      if (calibration_mode) {
        collecting_ = false;
        sample_s_ = 0.0;
      }
    } else {
      stationary_s_ += dt;
    }

    // During calibration the target output is zero. Wait until all previously
    // applied compensation AND commanded torques have decayed to ~zero before
    // taking a measurement, preventing feedback from learning its own output.
    if (calibration_mode && stationary && stationary_s_ >= settings_.stationary_dwell_s) {
      bool quiet = true;
      Joints residual{};
      for (std::size_t i = 0; i < kJoints; ++i) {
        residual[i] = measured_effort[i] - model_gravity[i];
        quiet = quiet && std::isfinite(residual[i]) &&
            std::abs(residual[i]) <= settings_.torque_limits[i] &&
            std::isfinite(last_command[i]) &&
            std::abs(last_command[i]) <= settings_.sampling_command_torque_epsilon &&
            std::abs(applied_[i]) <= settings_.sampling_command_torque_epsilon;
        if (collecting_) {
          quiet = quiet &&
              std::abs(residual[i] - filtered_sample_[i]) <= settings_.sample_deviation_limit;
        }
      }
      if (quiet) {
        if (!collecting_) {
          filtered_sample_ = residual;
          sample_anchor_ = q;
          collecting_ = true;
          sample_s_ = 0.0;
        } else {
          const double alpha = dt / (settings_.filter_tau_s + dt);
          for (std::size_t i = 0; i < kJoints; ++i) {
            filtered_sample_[i] += alpha * (residual[i] - filtered_sample_[i]);
          }
        }
        sample_s_ += dt;
      } else {
        collecting_ = false;
        sample_s_ = 0.0;
      }
    }

    // Outside calibration, continue a learned correction during hand guiding,
    // but fade it with pose distance (a static bias at q0 is not globally valid).
    double pose_scale = 0.0;
    if (valid_) {
      double max_distance = 0.0;
      for (std::size_t i = 0; i < kJoints; ++i) {
        max_distance = std::max(max_distance, std::abs(q[i] - anchor_[i]));
      }
      pose_scale = std::clamp(
          (settings_.pose_radius + settings_.pose_fade_width - max_distance) /
              settings_.pose_fade_width,
          0.0, 1.0);
    }
    const double max_step = settings_.output_slew_rate * dt;
    for (std::size_t i = 0; i < kJoints; ++i) {
      const double target = (command_fresh && !calibration_mode)
          ? estimate_[i] * pose_scale : 0.0;
      applied_[i] += std::clamp(target - applied_[i], -max_step, max_step);
      applied_[i] = std::clamp(applied_[i], -settings_.torque_limits[i],
                             settings_.torque_limits[i]);
    }
    return applied_;
  }

  const Joints& estimate() const { return estimate_; }
  const Joints& applied() const { return applied_; }
  bool calibrated() const { return valid_; }
  bool collecting() const { return collecting_; }
  double sample_seconds() const { return sample_s_; }

 private:
  Settings settings_;
  Joints estimate_{};
  Joints applied_{};
  Joints anchor_{};
  Joints filtered_sample_{};
  Joints sample_anchor_{};
  bool valid_{false};
  bool was_authorized_{false};
  bool collecting_{false};
  double stationary_s_{0.0};
  double sample_s_{0.0};
};

}  // namespace low_level
