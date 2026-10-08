#!/usr/bin/env python3

import bisect
import csv
from datetime import datetime, timezone
import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from ament_index_python.packages import get_package_prefix
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from stribeck_model import evaluate_stribeck, stribeck

try:
    from scipy.optimize import least_squares
except ImportError as exc:  # pragma: no cover - runtime environment dependent
    raise RuntimeError(
        'Friction identification requires scipy (python3-scipy). '
        'Install it with: sudo apt install python3-scipy'
    ) from exc


N = 7
TASK = 6
OBS_SIZE = 112


def _default_output_directory() -> str:
    """Return <colcon-workspace>/data/calibration when running from an installed package."""
    try:
        prefix = Path(get_package_prefix('low_level'))
        parts = prefix.parts
        install_indices = [i for i, part in enumerate(parts) if part == 'install']
        if install_indices:
            workspace_root = Path(*parts[:install_indices[-1]])
            return str(workspace_root / 'data' / 'calibration')
    except Exception:  # noqa: BLE001
        pass
    return str(Path.cwd() / 'data' / 'calibration')


def _as_float_array(value, name: str, size: int) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape != (size,):
        raise ValueError(f'{name} must contain exactly {size} values')
    if not np.all(np.isfinite(arr)):
        raise ValueError(f'{name} must contain only finite values')
    return arr


def _as_positive_float_array(value, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size == 0:
        raise ValueError(f'{name} must contain at least one value')
    if not np.all(np.isfinite(arr)) or np.any(arr <= 0.0):
        raise ValueError(f'{name} must contain only finite values > 0')
    return np.unique(np.sort(arr))


def _robust_location_scale(values: np.ndarray) -> Tuple[float, float]:
    """Median and MAD-based robust 1-sigma scale."""
    values = np.asarray(values, dtype=float).reshape(-1)
    if values.size == 0:
        raise ValueError('cannot compute a robust statistic from an empty array')
    median = float(np.median(values))
    mad_sigma = float(1.4826 * np.median(np.abs(values - median)))
    return median, mad_sigma


def _odd_friction_model(
    velocity: np.ndarray,
    coulomb: float,
    viscous: float,
    smoothing_velocity: float,
) -> np.ndarray:
    """Strictly odd Coulomb + viscous friction model.

    The model contains no constant/even torque term, so it cannot absorb
    gravity, payload, sensor-offset, or configuration-dependent residuals.
    """
    v = np.asarray(velocity, dtype=float)
    return coulomb * np.tanh(v / smoothing_velocity) + viscous * v


class DynamicsIdentificationExperiment(Node):
    def __init__(self) -> None:
        super().__init__('dynamics_identification_experiment')

        self.declare_parameter('robot_type', 'fr3')
        self.declare_parameter('arm_prefix', '')
        self.declare_parameter('motion_mode', 'joint')
        self.declare_parameter('command_topic', '/fr3/joint_commands')
        self.declare_parameter('twist_topic', '/fr3/cartesian_twist_command')
        self.declare_parameter('observation_topic', '/fr3/dynamics_observation')
        self.declare_parameter('root_frame', '')
        self.declare_parameter('run_motion', False)
        self.declare_parameter('command_rate', 100.0)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('stop_time', 2.0)
        self.declare_parameter('record_only_duration', 15.0)

        # Kept for launch-file backwards compatibility. Joint mode no longer uses
        # sine excitation; it uses the friction sweep below.
        self.declare_parameter('joint_duration', 8.0)
        self.declare_parameter(
            'velocity_amplitudes', [0.06, 0.06, 0.06, 0.05, 0.08, 0.08, 0.08]
        )
        self.declare_parameter(
            'frequencies', [0.13, 0.15, 0.17, 0.12, 0.19, 0.21, 0.23]
        )

        # Friction-only joint excitation. A +speed segment is immediately followed
        # by the corresponding -speed segment so the joint returns close to its
        # starting position after every speed level.
        self.declare_parameter(
            'friction_velocity_levels', [0.008, 0.012, 0.02, 0.03, 0.045, 0.065, 0.10, 0.15]
        )
        self.declare_parameter('friction_repeats', 4)
        self.declare_parameter('friction_ramp_time', 0.35)
        self.declare_parameter('friction_hold_time', 0.45)
        self.declare_parameter('friction_max_hold_time', 2.0)
        self.declare_parameter('friction_min_plateau_travel_rad', 0.01)
        self.declare_parameter('friction_pause_time', 0.15)
        self.declare_parameter('friction_plateau_trim_time', 0.10)
        self.declare_parameter('friction_min_fit_velocity', 0.004)
        self.declare_parameter('friction_speed_tolerance_ratio', 0.35)
        self.declare_parameter('friction_other_joint_max_drift_rad', 0.04)
        self.declare_parameter('friction_max_fit_acceleration', 0.05)
        # Legacy loss parameter retained for launch/backwards compatibility.
        self.declare_parameter('friction_robust_loss_scale_Nm', 0.08)
        self.declare_parameter('friction_smoothing_velocity', 0.01)
        self.declare_parameter('friction_weight_floor_Nm', 0.02)
        self.declare_parameter('friction_pair_grid_points', 15)
        self.declare_parameter('friction_min_pair_samples', 6)
        self.declare_parameter('friction_cross_repeat_warning_Nm', 0.09)
        self.declare_parameter('friction_plateau_noise_warning_Nm', 0.12)
        self.declare_parameter('friction_plateau_fit_rmse_warning_Nm', 0.08)

        # Legacy QP motion remains available so the launch interface is not broken,
        # but joint mode is the recommended friction calibration mode.
        self.declare_parameter('qp_axis_duration', 10.0)
        self.declare_parameter('qp_mixed_duration', 12.0)
        self.declare_parameter(
            'qp_twist_amplitudes', [0.12, 0.12, 0.10, 0.25, 0.25, 0.25]
        )
        self.declare_parameter(
            'qp_axis_frequencies', [0.20, 0.20, 0.20, 0.20, 0.20, 0.20]
        )
        self.declare_parameter('qp_mixed_scale', 0.45)

        self.declare_parameter('max_joint_displacement', 0.20)
        self.declare_parameter('max_measured_joint_velocity', 0.35)
        self.declare_parameter('max_measured_joint_acceleration', 3.0)
        self.declare_parameter('observation_timeout', 0.15)
        self.declare_parameter('output_directory', _default_output_directory())

        robot_type = str(self.get_parameter('robot_type').value)
        arm_prefix = str(self.get_parameter('arm_prefix').value)
        prefix = (arm_prefix + '_' if arm_prefix else '') + robot_type
        self.joint_names = [f'{prefix}_joint{i}' for i in range(1, 8)]

        self.motion_mode = str(self.get_parameter('motion_mode').value).strip().lower()
        if self.motion_mode not in ('joint', 'qp'):
            raise ValueError("motion_mode must be 'joint' or 'qp'")

        self.command_topic = str(self.get_parameter('command_topic').value)
        self.twist_topic = str(self.get_parameter('twist_topic').value)
        self.observation_topic = str(self.get_parameter('observation_topic').value)
        root_frame = str(self.get_parameter('root_frame').value).strip()
        self.root_frame = root_frame or f'{prefix}_link0'
        self.run_motion = bool(self.get_parameter('run_motion').value)
        self.command_rate = float(self.get_parameter('command_rate').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.stop_time = float(self.get_parameter('stop_time').value)
        self.record_only_duration = float(self.get_parameter('record_only_duration').value)

        self.friction_velocity_levels = _as_positive_float_array(
            self.get_parameter('friction_velocity_levels').value,
            'friction_velocity_levels',
        )
        self.friction_repeats = int(self.get_parameter('friction_repeats').value)
        self.friction_ramp_time = float(self.get_parameter('friction_ramp_time').value)
        self.friction_hold_time = float(self.get_parameter('friction_hold_time').value)
        self.friction_max_hold_time = float(self.get_parameter('friction_max_hold_time').value)
        self.friction_min_plateau_travel_rad = float(
            self.get_parameter('friction_min_plateau_travel_rad').value
        )
        self.friction_speed_tolerance_ratio = float(
            self.get_parameter('friction_speed_tolerance_ratio').value
        )
        self.friction_other_joint_max_drift_rad = float(
            self.get_parameter('friction_other_joint_max_drift_rad').value
        )
        self.friction_pause_time = float(self.get_parameter('friction_pause_time').value)
        self.friction_plateau_trim_time = float(
            self.get_parameter('friction_plateau_trim_time').value
        )
        self.friction_min_fit_velocity = float(
            self.get_parameter('friction_min_fit_velocity').value
        )
        self.friction_max_fit_acceleration = float(
            self.get_parameter('friction_max_fit_acceleration').value
        )
        self.friction_robust_loss_scale = float(
            self.get_parameter('friction_robust_loss_scale_Nm').value
        )
        self.friction_smoothing_velocity = float(
            self.get_parameter('friction_smoothing_velocity').value
        )
        self.friction_weight_floor = float(
            self.get_parameter('friction_weight_floor_Nm').value
        )
        self.friction_pair_grid_points = int(
            self.get_parameter('friction_pair_grid_points').value
        )
        self.friction_min_pair_samples = int(
            self.get_parameter('friction_min_pair_samples').value
        )
        self.friction_cross_repeat_warning = float(
            self.get_parameter('friction_cross_repeat_warning_Nm').value
        )
        self.friction_plateau_noise_warning = float(
            self.get_parameter('friction_plateau_noise_warning_Nm').value
        )
        self.friction_plateau_fit_rmse_warning = float(
            self.get_parameter('friction_plateau_fit_rmse_warning_Nm').value
        )

        self.qp_axis_duration = float(self.get_parameter('qp_axis_duration').value)
        self.qp_mixed_duration = float(self.get_parameter('qp_mixed_duration').value)
        self.qp_twist_amplitudes = _as_float_array(
            self.get_parameter('qp_twist_amplitudes').value, 'qp_twist_amplitudes', TASK
        )
        self.qp_axis_frequencies = _as_float_array(
            self.get_parameter('qp_axis_frequencies').value, 'qp_axis_frequencies', TASK
        )
        self.qp_mixed_scale = float(self.get_parameter('qp_mixed_scale').value)

        self.max_joint_displacement = float(
            self.get_parameter('max_joint_displacement').value
        )
        self.max_measured_joint_velocity = float(
            self.get_parameter('max_measured_joint_velocity').value
        )
        self.max_measured_joint_acceleration = float(
            self.get_parameter('max_measured_joint_acceleration').value
        )
        self.observation_timeout = float(self.get_parameter('observation_timeout').value)
        self.output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser()

        if self.command_rate <= 0.0:
            raise ValueError('command_rate must be > 0')
        if self.settle_time < 0.0 or self.stop_time < 0.0:
            raise ValueError('settle_time/stop_time must be >= 0')
        if self.record_only_duration <= 0.0:
            raise ValueError('record_only_duration must be > 0')
        if min(
            self.friction_ramp_time,
            self.friction_hold_time,
            self.friction_max_hold_time,
            self.friction_min_plateau_travel_rad,
            self.friction_speed_tolerance_ratio,
            self.friction_other_joint_max_drift_rad,
            self.friction_pause_time,
            self.friction_min_fit_velocity,
            self.friction_max_fit_acceleration,
            self.friction_robust_loss_scale,
            self.friction_smoothing_velocity,
            self.friction_weight_floor,
            self.friction_cross_repeat_warning,
            self.friction_plateau_noise_warning,
            self.friction_plateau_fit_rmse_warning,
            self.max_joint_displacement,
            self.max_measured_joint_velocity,
            self.max_measured_joint_acceleration,
            self.observation_timeout,
        ) <= 0.0:
            raise ValueError('friction timing/fit/safety parameters must be > 0')
        if self.friction_repeats < 3:
            raise ValueError('Stribeck identification requires at least 3 independent repeats')
        if self.friction_hold_time > self.friction_max_hold_time:
            raise ValueError('friction_hold_time cannot exceed friction_max_hold_time')
        if len(self.friction_velocity_levels) < 6:
            raise ValueError('Stribeck identification requires at least six distinct velocities')
        if self.friction_velocity_levels[0] > 0.015:
            raise ValueError('Stribeck identification requires a low-speed level <= 0.015 rad/s')
        if self.friction_velocity_levels[-1] < 0.08:
            raise ValueError('Stribeck identification requires an upper speed level >= 0.08 rad/s')
        if self.friction_pair_grid_points < 5:
            raise ValueError('friction_pair_grid_points must be >= 5')
        if self.friction_min_pair_samples < 5:
            raise ValueError('friction_min_pair_samples must be >= 5')
        if self.friction_plateau_trim_time < 0.0:
            raise ValueError('friction_plateau_trim_time must be >= 0')
        if 2.0 * self.friction_plateau_trim_time >= self.friction_hold_time:
            raise ValueError('friction_plateau_trim_time leaves no usable hold interval')
        if any(
            speed * (
                self.friction_ramp_time + min(
                    self.friction_max_hold_time,
                    max(self.friction_hold_time, self.friction_min_plateau_travel_rad / speed)
                )
            ) > 0.9 * self.max_joint_displacement
            for speed in self.friction_velocity_levels
        ):
            raise ValueError('sweep would exceed 90% of max_joint_displacement; adjust speed or timing')
        if np.any(self.qp_twist_amplitudes < 0.0) or np.any(self.qp_axis_frequencies <= 0.0):
            raise ValueError('QP twist amplitudes must be >= 0 and frequencies > 0')
        if not 0.0 <= self.qp_mixed_scale <= 1.0:
            raise ValueError('qp_mixed_scale must be between 0 and 1')

        joint_qos = rclpy.qos.QoSProfile(depth=1)
        self.joint_publisher = self.create_publisher(JointState, self.command_topic, joint_qos)

        twist_qos = rclpy.qos.QoSProfile(depth=1)
        twist_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        twist_qos.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE
        self.twist_publisher = self.create_publisher(TwistStamped, self.twist_topic, twist_qos)

        self.subscription = self.create_subscription(
            Float64MultiArray,
            self.observation_topic,
            self._observation_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / self.command_rate, self._timer_callback)

        self.records: List[Dict[str, object]] = []
        self.started_at = None
        self.last_observation_at = None
        self.start_q = None
        self.finished = False
        self.abort_reason = None
        self.zero_started_at = None

        self.current_joint_command = np.zeros(N)
        self.current_fit_joint = -1
        self.current_fit_repeat = -1
        self.current_fit_active = False
        self.current_target_speed = 0.0
        self.friction_schedule = self._build_friction_schedule()
        self.friction_schedule_ends = [item['end'] for item in self.friction_schedule]

        mode = 'ACTIVE MOTION' if self.run_motion else 'record-only'
        self.get_logger().info(
            f'Friction identification initialized in {mode}, motion_mode={self.motion_mode}. '
            f'Franka M/C/g are fixed and only the residual friction model is identified.'
        )
        self.get_logger().info(f'Calibration data directory: {self.output_directory}')
        if self.run_motion and self.motion_mode == 'joint':
            speeds = ', '.join(f'{v:.3f}' for v in self.friction_velocity_levels)
            self.get_logger().warn(
                'Friction sweep enabled: one joint at a time, both directions, constant-speed '
                f'plateaus at [{speeds}] rad/s, {self.friction_repeats} repeat(s). '
                'Single initial robot configuration; odd repeats reverse order to expose drift. '
                f'Estimated sweep duration: {self.friction_schedule_ends[-1] / 60.0:.1f} min. '
                'Keep the entire robot workspace clear.'
            )
        elif self.run_motion:
            self.get_logger().warn(
                'QP motion selected. Friction will be fitted only from low-acceleration samples; '
                'joint friction sweep mode is preferred for calibration.'
            )

    def _observation_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != OBS_SIZE:
            self.get_logger().error(
                f'Expected {OBS_SIZE} observer values, received {len(msg.data)}'
            )
            return
        data = np.asarray(msg.data, dtype=float)
        if not np.all(np.isfinite(data)):
            return

        now = time.monotonic()
        q = data[0:7].copy()
        dq = data[7:14].copy()
        ddq = data[14:21].copy()
        tau_measured = data[21:28].copy()
        tau_inertia = data[28:35].copy()
        coriolis = data[35:42].copy()
        gravity = data[42:49].copy()
        tau_model = data[49:56].copy()
        residual = data[56:63].copy()
        mass = data[63:112].reshape((7, 7), order='F').copy()

        if self.started_at is None:
            self.started_at = now
            self.start_q = q.copy()
            self.get_logger().info('First dynamics observation received; experiment timing starts now.')
        self.last_observation_at = now

        self.records.append({
            't': now,
            'q': q,
            'dq': dq,
            'ddq': ddq,
            'tau_measured': tau_measured,
            'tau_inertia': tau_inertia,
            'coriolis': coriolis,
            'gravity': gravity,
            'tau_model': tau_model,
            'residual': residual,
            'mass': mass,
            'command_velocity': self.current_joint_command.copy(),
            'fit_joint': int(self.current_fit_joint),
            'fit_repeat': int(self.current_fit_repeat),
            'fit_active': bool(self.current_fit_active),
            'target_speed': float(self.current_target_speed),
        })

        if self.run_motion and self.abort_reason is None:
            displacement = np.abs(q - self.start_q)
            if self.motion_mode == 'joint' and self.current_fit_joint >= 0:
                drift = displacement.copy()
                drift[self.current_fit_joint] = 0.0
                if float(np.max(drift)) > self.friction_other_joint_max_drift_rad:
                    k = int(np.argmax(drift))
                    self._abort(
                        f'non-excited joint {k + 1} drifted {drift[k]:.3f} rad from '
                        f'the calibration start pose (limit {self.friction_other_joint_max_drift_rad:.3f})'
                    )
                    return
            if np.max(displacement) > self.max_joint_displacement:
                j = int(np.argmax(displacement))
                self._abort(
                    f'joint {j + 1} displacement {displacement[j]:.3f} rad exceeded '
                    f'{self.max_joint_displacement:.3f} rad guard'
                )
                return
            if np.max(np.abs(dq)) > self.max_measured_joint_velocity:
                j = int(np.argmax(np.abs(dq)))
                self._abort(
                    f'joint {j + 1} velocity {dq[j]:.3f} rad/s exceeded '
                    f'{self.max_measured_joint_velocity:.3f} rad/s guard'
                )
                return
            if np.max(np.abs(ddq)) > self.max_measured_joint_acceleration:
                j = int(np.argmax(np.abs(ddq)))
                self._abort(
                    f'joint {j + 1} acceleration {ddq[j]:.3f} rad/s^2 exceeded '
                    f'{self.max_measured_joint_acceleration:.3f} rad/s^2 guard'
                )

    def _abort(self, reason: str) -> None:
        if self.abort_reason is not None:
            return
        self.abort_reason = reason
        self.zero_started_at = time.monotonic()
        self.current_fit_active = False
        self.current_fit_joint = -1
        self.current_fit_repeat = -1
        self.current_target_speed = 0.0
        self.get_logger().error('Aborting excitation: ' + reason)

    def _publish_joint_velocity(self, velocity: np.ndarray) -> None:
        self.current_joint_command = np.asarray(velocity, dtype=float).copy()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.velocity = [float(v) for v in velocity]
        self.joint_publisher.publish(msg)

    def _publish_twist(self, twist: np.ndarray) -> None:
        self.current_joint_command[:] = 0.0
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.root_frame
        msg.twist.linear.x = float(twist[0])
        msg.twist.linear.y = float(twist[1])
        msg.twist.linear.z = float(twist[2])
        msg.twist.angular.x = float(twist[3])
        msg.twist.angular.y = float(twist[4])
        msg.twist.angular.z = float(twist[5])
        self.twist_publisher.publish(msg)

    def _publish_zero(self) -> None:
        self.current_fit_active = False
        self.current_fit_joint = -1
        self.current_fit_repeat = -1
        self.current_target_speed = 0.0
        if self.motion_mode == 'qp':
            self._publish_twist(np.zeros(TASK))
        else:
            self._publish_joint_velocity(np.zeros(N))

    def _build_friction_schedule(self) -> List[Dict[str, float]]:
        """Sequence +/- sweeps around ONE initial pose; adapt hold time to speed.

        Longer low-speed plateaus provide measurable displacement and enough
        position-matched samples without demanding large high-speed travel.
        This does NOT mean identifying friction at exactly stationary q.
        """
        result = []
        t = 0.0
        for joint in range(N):
            for repeat in range(self.friction_repeats):
                speeds = (self.friction_velocity_levels if repeat % 2 == 0
                          else self.friction_velocity_levels[::-1])
                directions = (1.0, -1.0) if repeat % 2 == 0 else (-1.0, 1.0)
                for speed in speeds:
                    hold = min(self.friction_max_hold_time,
                               max(self.friction_hold_time,
                                   self.friction_min_plateau_travel_rad / float(speed)))
                    for direction in directions:
                        duration = 2.0 * self.friction_ramp_time + hold + self.friction_pause_time
                        result.append({
                            'start': t, 'end': t + duration,
                            'joint': joint, 'repeat': repeat,
                            'target': float(direction * speed), 'hold': hold,
                        })
                        t += duration
        return result

    def _friction_excitation(self, elapsed: float) -> Tuple[np.ndarray, bool]:
        """Return ramp/plateau/deceleration commands for one-start-pose sweeps."""
        idx = bisect.bisect_right(self.friction_schedule_ends, elapsed)
        if idx >= len(self.friction_schedule):
            self.current_fit_active = False
            self.current_fit_joint = -1
            self.current_fit_repeat = -1
            self.current_target_speed = 0.0
            return np.zeros(N), True

        segment = self.friction_schedule[idx]
        joint_index = int(segment['joint'])
        repeat_index = int(segment['repeat'])
        target = float(segment['target'])
        hold = float(segment['hold'])
        local_t = elapsed - float(segment['start'])
        ramp = self.friction_ramp_time
        command = 0.0
        fit_active = False
        if local_t < ramp:
            u = max(0.0, min(1.0, local_t / ramp))
            command = target * 0.5 * (1.0 - math.cos(math.pi * u))
        elif local_t < ramp + hold:
            command = target
            hold_t = local_t - ramp
            fit_active = (
                self.friction_plateau_trim_time <= hold_t
                <= hold - self.friction_plateau_trim_time
            )
        elif local_t < 2.0 * ramp + hold:
            u = max(0.0, min(1.0, (local_t - ramp - hold) / ramp))
            command = target * 0.5 * (1.0 + math.cos(math.pi * u))

        velocity = np.zeros(N)
        velocity[joint_index] = command
        self.current_fit_joint = joint_index
        self.current_fit_repeat = repeat_index
        self.current_fit_active = fit_active
        self.current_target_speed = target if fit_active else 0.0
        return velocity, False

    def _qp_sequential_twist(self, elapsed: float) -> np.ndarray:
        axis = int(elapsed // self.qp_axis_duration)
        local_t = elapsed - axis * self.qp_axis_duration
        envelope = math.sin(math.pi * local_t / self.qp_axis_duration) ** 2
        twist = np.zeros(TASK)
        twist[axis] = (
            self.qp_twist_amplitudes[axis]
            * math.sin(2.0 * math.pi * self.qp_axis_frequencies[axis] * local_t)
            * envelope
        )
        return twist

    def _qp_mixed_twist(self, elapsed: float) -> np.ndarray:
        if self.qp_mixed_duration <= 0.0:
            return np.zeros(TASK)
        envelope = math.sin(math.pi * elapsed / self.qp_mixed_duration) ** 2
        cycles = np.asarray([2.0, 3.0, 2.0, 2.0, 3.0, 2.0])
        frequencies = cycles / self.qp_mixed_duration
        return (
            self.qp_mixed_scale
            * self.qp_twist_amplitudes
            * np.sin(2.0 * math.pi * frequencies * elapsed)
            * envelope
        )

    def _timer_callback(self) -> None:
        if self.finished or self.started_at is None:
            return

        now = time.monotonic()
        if self.last_observation_at is None or now - self.last_observation_at > self.observation_timeout:
            if self.run_motion:
                stale = now - (self.last_observation_at or now)
                self._abort(f'dynamics observation stale for {stale:.3f} s')

        if self.abort_reason is not None:
            self._publish_zero()
            if self.zero_started_at is not None and now - self.zero_started_at >= self.stop_time:
                self._finish()
            return

        elapsed = now - self.started_at
        if not self.run_motion:
            if elapsed >= self.record_only_duration:
                self._finish()
            return

        if elapsed < self.settle_time:
            self._publish_zero()
            return

        motion_t = elapsed - self.settle_time
        if self.motion_mode == 'joint':
            velocity, complete = self._friction_excitation(motion_t)
            if not complete:
                self._publish_joint_velocity(velocity)
                return
        else:
            self.current_fit_active = False
            self.current_fit_joint = -1
            self.current_fit_repeat = -1
            self.current_target_speed = 0.0
            sequential = TASK * self.qp_axis_duration
            if motion_t < sequential:
                self._publish_twist(self._qp_sequential_twist(motion_t))
                return
            mixed_t = motion_t - sequential
            if mixed_t < self.qp_mixed_duration:
                self._publish_twist(self._qp_mixed_twist(mixed_t))
                return

        self._publish_zero()
        if self.zero_started_at is None:
            self.zero_started_at = now
            self.get_logger().info('Excitation complete; holding zero command before saving.')
        elif now - self.zero_started_at >= self.stop_time:
            self._finish()

    def _finish(self) -> None:
        if self.finished:
            return
        self.finished = True
        self._publish_zero()
        try:
            csv_path, json_path, calibration_path = self._save_and_fit()
            self.get_logger().info(f'Friction data saved to {csv_path}')
            self.get_logger().info(f'Friction identification summary saved to {json_path}')
            if calibration_path is not None:
                self.get_logger().info(f'Controller-ready friction calibration saved to {calibration_path}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to save friction identification result: {exc}')
        self.timer.cancel()
        rclpy.shutdown()

    @staticmethod
    def _interpolate_on_position_grid(
        q: np.ndarray, values: np.ndarray, grid: np.ndarray
    ) -> np.ndarray:
        order = np.argsort(q)
        q_sorted = np.asarray(q, dtype=float)[order]
        values_sorted = np.asarray(values, dtype=float)[order]
        q_unique, unique_index = np.unique(q_sorted, return_index=True)
        if q_unique.size < 2:
            raise RuntimeError('not enough unique joint positions for matched pairing')
        return np.interp(grid, q_unique, values_sorted[unique_index])

    def _build_position_matched_pairs(
        self,
        joint_i: int,
        q: np.ndarray,
        dq: np.ndarray,
        residual: np.ndarray,
        fit_repeat: np.ndarray,
        target_speed: np.ndarray,
        usable_mask: np.ndarray,
    ) -> List[Dict[str, object]]:
        """Build matched +v/-v observations at the same joint positions.

        The odd component isolates friction while the even component is retained
        only as a diagnostic for payload/model/sensor/configuration residuals.
        """
        paired: List[Dict[str, object]] = []
        for repeat_index in range(self.friction_repeats):
            for speed in self.friction_velocity_levels:
                pos_mask = (
                    usable_mask
                    & (fit_repeat == repeat_index)
                    & np.isclose(target_speed, float(speed), atol=1e-8)
                )
                neg_mask = (
                    usable_mask
                    & (fit_repeat == repeat_index)
                    & np.isclose(target_speed, -float(speed), atol=1e-8)
                )
                if (
                    np.count_nonzero(pos_mask) < self.friction_min_pair_samples
                    or np.count_nonzero(neg_mask) < self.friction_min_pair_samples
                ):
                    continue

                q_pos = q[pos_mask, joint_i]
                q_neg = q[neg_mask, joint_i]
                r_pos = residual[pos_mask, joint_i]
                r_neg = residual[neg_mask, joint_i]
                v_pos = np.abs(dq[pos_mask, joint_i])
                v_neg = np.abs(dq[neg_mask, joint_i])

                q_lo = max(float(np.min(q_pos)), float(np.min(q_neg)))
                q_hi = min(float(np.max(q_pos)), float(np.max(q_neg)))
                if q_hi <= q_lo + 1e-5:
                    continue

                grid_count = min(
                    self.friction_pair_grid_points,
                    int(np.unique(q_pos).size),
                    int(np.unique(q_neg).size),
                )
                if grid_count < self.friction_min_pair_samples:
                    continue
                grid = np.linspace(q_lo, q_hi, grid_count)

                rp = self._interpolate_on_position_grid(q_pos, r_pos, grid)
                rn = self._interpolate_on_position_grid(q_neg, r_neg, grid)
                vp = self._interpolate_on_position_grid(q_pos, v_pos, grid)
                vn = self._interpolate_on_position_grid(q_neg, v_neg, grid)

                odd_samples = 0.5 * (rp - rn)
                even_samples = 0.5 * (rp + rn)
                speed_samples = 0.5 * (vp + vn)

                odd_median, odd_mad = _robust_location_scale(odd_samples)
                even_median, even_mad = _robust_location_scale(even_samples)
                speed_median, speed_mad = _robust_location_scale(speed_samples)
                uncertainty = max(self.friction_weight_floor, odd_mad)

                paired.append({
                    'repeat': repeat_index + 1,
                    'target_speed_rad_s': float(speed),
                    'samples_positive': int(np.count_nonzero(pos_mask)),
                    'samples_negative': int(np.count_nonzero(neg_mask)),
                    'matched_grid_points': int(grid_count),
                    'q_overlap_min_rad': q_lo,
                    'q_overlap_max_rad': q_hi,
                    'q_overlap_span_rad': q_hi - q_lo,
                    'matched_speed_median_rad_s': speed_median,
                    'matched_speed_mad_sigma_rad_s': speed_mad,
                    'odd_friction_median_Nm': odd_median,
                    'odd_friction_mad_sigma_Nm': odd_mad,
                    'even_residual_median_Nm': even_median,
                    'even_residual_mad_sigma_Nm': even_mad,
                    'fit_uncertainty_Nm': uncertainty,
                })
        return paired

    def _fit_odd_pair_model(
        self, paired: List[Dict[str, object]]
    ) -> Tuple[np.ndarray, object, np.ndarray]:
        if len(paired) < 6:
            raise RuntimeError('at least six matched +/- observations are required')

        velocity = np.asarray(
            [float(p['matched_speed_median_rad_s']) for p in paired], dtype=float
        )
        odd_torque = np.asarray(
            [float(p['odd_friction_median_Nm']) for p in paired], dtype=float
        )
        uncertainty = np.asarray(
            [max(self.friction_weight_floor, float(p['fit_uncertainty_Nm'])) for p in paired],
            dtype=float,
        )

        low_speed_order = np.argsort(velocity)
        low_count = max(3, min(6, velocity.size // 3))
        fc0 = max(0.02, float(np.median(odd_torque[low_speed_order[:low_count]])))
        if velocity.size >= 2:
            dv = max(float(np.max(velocity) - np.min(velocity)), 1e-6)
            b0 = max(0.0, float((np.max(odd_torque) - np.min(odd_torque)) / dv))
        else:
            b0 = 0.2
        x0 = np.asarray([min(fc0, 2.0), min(b0, 5.0)])

        def normalized_error(params: np.ndarray) -> np.ndarray:
            prediction = _odd_friction_model(
                velocity, float(params[0]), float(params[1]), self.friction_smoothing_velocity
            )
            return (prediction - odd_torque) / uncertainty

        result = least_squares(
            normalized_error,
            x0,
            bounds=(np.asarray([0.0, 0.0]), np.asarray([3.0, 10.0])),
            loss='soft_l1',
            f_scale=1.5,
            max_nfev=4000,
        )
        return result.x, result, uncertainty

    def _fit_joint_friction_from_pairs(
        self, paired: List[Dict[str, object]]
    ) -> Tuple[Dict[str, object], Dict[str, float]]:
        """Fit the odd friction model on the largest repeatable speed band.

        High-speed plateaus on the proximal FR3 joints can become dominated by
        observer/acceleration/model residual noise even when the lower-speed
        friction curve is highly repeatable.  Rejecting the entire joint in that
        case throws away useful calibration data.  Instead, evaluate contiguous
        speed bands starting at the lowest requested speed and select the widest
        band whose fit and leave-one-repeat-out validation are at least usable.
        The controller then clamps compensation at that joint-specific validated
        maximum speed.
        """
        if len(paired) < 6:
            raise RuntimeError('at least six matched +/- observations are required')

        all_paired = paired

        # A speed level is eligible for the validated band only if it is observed
        # in enough independent repeats.  With the default four repeats require
        # three; preserve sensible behavior if the user deliberately requests
        # fewer repeats.
        required_repeats = max(1, min(3, self.friction_repeats))
        speed_repeat_counts: Dict[float, int] = {}
        for speed in self.friction_velocity_levels:
            speed_repeat_counts[float(speed)] = len({
                int(p['repeat']) for p in all_paired
                if abs(float(p['target_speed_rad_s']) - float(speed)) < 1e-9
            })

        contiguous_speeds: List[float] = []
        for speed in self.friction_velocity_levels:
            speed_f = float(speed)
            if speed_repeat_counts.get(speed_f, 0) >= required_repeats:
                contiguous_speeds.append(speed_f)
            else:
                break

        if not contiguous_speeds:
            # Fall back to whatever data exist.  The quality gate below will keep
            # an under-supported fit disabled rather than silently accepting it.
            contiguous_speeds = sorted({
                float(p['target_speed_rad_s']) for p in all_paired
            })

        def classify_quality(
            odd_rmse: float,
            median_pair_noise: float,
            cross_repeat_rmse: Optional[float],
            cross_repeat_max: Optional[float],
        ) -> str:
            good_validation = (
                cross_repeat_rmse is None or cross_repeat_rmse <= 0.05
            ) and (cross_repeat_max is None or cross_repeat_max <= 0.08)
            usable_validation = (
                cross_repeat_rmse is None or cross_repeat_rmse <= 0.09
            ) and (cross_repeat_max is None or cross_repeat_max <= 0.13)
            if odd_rmse <= 0.04 and median_pair_noise <= 0.08 and good_validation:
                return 'good'
            if odd_rmse <= 0.08 and median_pair_noise <= 0.15 and usable_validation:
                return 'usable'
            return 'poor'

        def evaluate_subset(
            subset: List[Dict[str, object]],
        ) -> Dict[str, object]:
            params, result, uncertainty = self._fit_odd_pair_model(subset)
            coulomb, viscous = [float(x) for x in params]
            velocity = np.asarray(
                [float(p['matched_speed_median_rad_s']) for p in subset], dtype=float
            )
            truth = np.asarray(
                [float(p['odd_friction_median_Nm']) for p in subset], dtype=float
            )
            prediction = _odd_friction_model(
                velocity, coulomb, viscous, self.friction_smoothing_velocity
            )
            odd_rmse = float(np.sqrt(np.mean((truth - prediction) ** 2)))
            normalized_rmse = float(
                np.sqrt(np.mean(((truth - prediction) / uncertainty) ** 2))
            )
            odd_rms = float(np.sqrt(np.mean(truth ** 2)))
            odd_improvement = 100.0 * (1.0 - odd_rmse / max(odd_rms, 1e-12))
            median_pair_noise = float(np.median(uncertainty))
            max_pair_noise = float(np.max(uncertainty))

            repeats = sorted({int(p['repeat']) for p in subset})
            cross_repeat = []
            if len(repeats) >= 2:
                for held_repeat in repeats:
                    train = [p for p in subset if int(p['repeat']) != held_repeat]
                    test = [p for p in subset if int(p['repeat']) == held_repeat]
                    if len(train) < 6 or not test:
                        continue
                    held_params, held_result, _ = self._fit_odd_pair_model(train)
                    test_v = np.asarray(
                        [float(p['matched_speed_median_rad_s']) for p in test], dtype=float
                    )
                    test_y = np.asarray(
                        [float(p['odd_friction_median_Nm']) for p in test], dtype=float
                    )
                    test_pred = _odd_friction_model(
                        test_v, float(held_params[0]), float(held_params[1]),
                        self.friction_smoothing_velocity
                    )
                    cross_repeat.append({
                        'held_repeat': held_repeat,
                        'fit_success': bool(held_result.success),
                        'validation_rmse_Nm': float(
                            np.sqrt(np.mean((test_y - test_pred) ** 2))
                        ),
                        'trained_coulomb_Nm': float(held_params[0]),
                        'trained_viscous_Nm_per_rad_s': float(held_params[1]),
                    })
            cross_repeat_rmse = (
                float(np.mean([x['validation_rmse_Nm'] for x in cross_repeat]))
                if cross_repeat else None
            )
            cross_repeat_max = (
                float(np.max([x['validation_rmse_Nm'] for x in cross_repeat]))
                if cross_repeat else None
            )
            quality = classify_quality(
                odd_rmse, median_pair_noise, cross_repeat_rmse, cross_repeat_max
            )
            return {
                'params': params,
                'result': result,
                'uncertainty': uncertainty,
                'velocity': velocity,
                'truth': truth,
                'prediction': prediction,
                'odd_rmse': odd_rmse,
                'normalized_rmse': normalized_rmse,
                'odd_rms': odd_rms,
                'odd_improvement': odd_improvement,
                'median_pair_noise': median_pair_noise,
                'max_pair_noise': max_pair_noise,
                'cross_repeat': cross_repeat,
                'cross_repeat_rmse': cross_repeat_rmse,
                'cross_repeat_max': cross_repeat_max,
                'quality': quality,
            }

        # Require at least three distinct speeds to identify Coulomb and viscous
        # terms robustly.  Search widest-to-narrowest and keep the first band that
        # passes the existing quality gates.  This does not relax any threshold.
        candidate_maxima = contiguous_speeds[:]
        candidate_diagnostics: List[Dict[str, object]] = []
        selected_subset: Optional[List[Dict[str, object]]] = None
        selected_eval: Optional[Dict[str, object]] = None
        selected_max: Optional[float] = None

        for candidate_max in reversed(candidate_maxima):
            subset = [
                p for p in all_paired
                if float(p['target_speed_rad_s']) <= candidate_max + 1e-9
            ]
            distinct_speeds = sorted({float(p['target_speed_rad_s']) for p in subset})
            if len(distinct_speeds) < 3 or len(subset) < 6:
                continue
            evaluation = evaluate_subset(subset)
            candidate_diagnostics.append({
                'velocity_max_rad_s': candidate_max,
                'matched_pair_groups': len(subset),
                'odd_component_fit_rmse_Nm': evaluation['odd_rmse'],
                'median_pair_uncertainty_Nm': evaluation['median_pair_noise'],
                'cross_repeat_validation_rmse_Nm': evaluation['cross_repeat_rmse'],
                'cross_repeat_validation_max_rmse_Nm': evaluation['cross_repeat_max'],
                'quality': evaluation['quality'],
            })
            if evaluation['quality'] != 'poor':
                selected_subset = subset
                selected_eval = evaluation
                selected_max = candidate_max
                break

        if selected_subset is None or selected_eval is None or selected_max is None:
            # No candidate passed.  Use the widest contiguous band for diagnostics
            # and leave the joint marked poor/disabled.
            selected_max = float(contiguous_speeds[-1])
            selected_subset = [
                p for p in all_paired
                if float(p['target_speed_rad_s']) <= selected_max + 1e-9
            ]
            if len(selected_subset) < 6:
                selected_subset = all_paired
                selected_max = float(max(float(p['target_speed_rad_s']) for p in all_paired))
            selected_eval = evaluate_subset(selected_subset)

        params = np.asarray(selected_eval['params'], dtype=float)
        result = selected_eval['result']
        uncertainty = np.asarray(selected_eval['uncertainty'], dtype=float)
        coulomb, viscous = [float(x) for x in params]
        odd_rmse = float(selected_eval['odd_rmse'])
        normalized_rmse = float(selected_eval['normalized_rmse'])
        odd_rms = float(selected_eval['odd_rms'])
        odd_improvement = float(selected_eval['odd_improvement'])
        median_pair_noise = float(selected_eval['median_pair_noise'])
        max_pair_noise = float(selected_eval['max_pair_noise'])
        cross_repeat = selected_eval['cross_repeat']
        cross_repeat_rmse = selected_eval['cross_repeat_rmse']
        cross_repeat_max = selected_eval['cross_repeat_max']
        quality = str(selected_eval['quality'])

        selected_ids = {id(p) for p in selected_subset}
        for item in all_paired:
            item['used_for_fit'] = id(item) in selected_ids
            v = float(item['matched_speed_median_rad_s'])
            predicted = float(_odd_friction_model(
                np.asarray([v], dtype=float), coulomb, viscous,
                self.friction_smoothing_velocity
            )[0])
            item['fitted_odd_friction_Nm'] = predicted
            item['odd_fit_error_Nm'] = float(item['odd_friction_median_Nm']) - predicted

        selected_even_values = np.asarray(
            [float(p['even_residual_median_Nm']) for p in selected_subset], dtype=float
        )
        bias_diagnostic, even_spread = _robust_location_scale(selected_even_values)

        parameter_std = [float('nan')] * 2
        try:
            jac = np.asarray(result.jac, dtype=float)
            dof = max(1, jac.shape[0] - jac.shape[1])
            scale2 = float(2.0 * result.cost / dof)
            covariance = np.linalg.pinv(jac.T @ jac) * scale2
            parameter_std = [float(max(0.0, covariance[i, i]) ** 0.5) for i in range(2)]
        except Exception:  # noqa: BLE001
            pass

        selected_speeds = sorted({
            float(p['target_speed_rad_s']) for p in selected_subset
        })
        validated_min = float(min(selected_speeds))
        validated_max = float(max(selected_speeds))

        warnings = []
        if median_pair_noise > self.friction_plateau_noise_warning:
            warnings.append('position-matched odd friction observations are noisy')
        if odd_rmse > self.friction_plateau_fit_rmse_warning:
            warnings.append('odd Coulomb+viscous model fit error is high')
        if cross_repeat_rmse is not None and cross_repeat_rmse > self.friction_cross_repeat_warning:
            warnings.append('odd friction calibration is not repeatable across repeats')
        if even_spread > 0.10:
            warnings.append('large even residual remains; it is excluded from friction compensation')
        requested_max = float(np.max(self.friction_velocity_levels))
        if validated_max < requested_max - 1e-9:
            warnings.append(
                f'automatically limited validated speed to {validated_max:.3f} rad/s; '
                'higher-speed matched pairs failed support/fit/repeatability checks'
            )

        details: Dict[str, object] = {
            'solver_success': bool(result.success),
            'solver_message': str(result.message),
            'matched_pair_groups': int(len(all_paired)),
            'matched_pair_groups_used': int(len(selected_subset)),
            'required_repeats_per_speed_for_validated_band': required_repeats,
            'speed_repeat_counts': {
                f'{speed:.6g}': int(count) for speed, count in speed_repeat_counts.items()
            },
            'velocity_range_candidates': candidate_diagnostics,
            'torque_bias_diagnostic_Nm': bias_diagnostic,
            'even_residual_mad_sigma_Nm': even_spread,
            'coulomb_friction_Nm': coulomb,
            'viscous_friction_Nm_per_rad_s': viscous,
            'parameter_std_approx': {
                'coulomb_friction_Nm': parameter_std[0],
                'viscous_friction_Nm_per_rad_s': parameter_std[1],
            },
            'smoothing_velocity_rad_s': self.friction_smoothing_velocity,
            'odd_component_rms_Nm': odd_rms,
            'odd_component_fit_rmse_Nm': odd_rmse,
            'odd_component_rmse_improvement_percent': odd_improvement,
            'normalized_odd_fit_rmse_sigma': normalized_rmse,
            'median_pair_uncertainty_Nm': median_pair_noise,
            'max_pair_uncertainty_Nm': max_pair_noise,
            'cross_repeat_validation': cross_repeat,
            'cross_repeat_validation_rmse_Nm': cross_repeat_rmse,
            'cross_repeat_validation_max_rmse_Nm': cross_repeat_max,
            'validated_velocity_min_rad_s': validated_min,
            'validated_velocity_max_rad_s': validated_max,
            'position_matched_pairs': all_paired,
            'quality': quality,
            'warnings': warnings,
        }
        compact = {
            'bias': bias_diagnostic,
            'coulomb': coulomb,
            'viscous': viscous,
            'velocity_min': validated_min,
            'velocity_max': validated_max,
        }
        return details, compact

    def _fit_stribeck_or_fallback(self, paired, baseline_details, baseline_compact):
        """Prefer validated Stribeck; fall back to Fc+Bdq when no CV evidence.

        Fallback is still represented as a schema-v4 Stribeck model with Fs=Fc.
        No fabricated static-friction peak is ever enabled.
        """
        details = baseline_details
        compact = baseline_compact
        counts = details['speed_repeat_counts']
        required = int(details['required_repeats_per_speed_for_validated_band'])
        available = []
        for speed in self.friction_velocity_levels:
            if int(counts.get(f'{speed:.6g}', 0)) < required:
                break
            available.append(float(speed))

        diagnostics = []
        selected = None
        selected_pairs = None
        for vmax in reversed(available):
            subset = [p for p in paired if float(p['target_speed_rad_s']) <= vmax + 1e-9]
            unique_levels = {float(p['target_speed_rad_s']) for p in subset}
            if len(unique_levels) < 6 or min(unique_levels) > 0.015:
                continue
            try:
                result = evaluate_stribeck(
                    subset, self.friction_smoothing_velocity,
                    self.friction_weight_floor,
                )
                result['candidate_velocity_max_rad_s'] = vmax
                diagnostics.append(result)
                if result['selected']:
                    selected, selected_pairs = result, subset
                    break
            except (RuntimeError, ValueError) as exc:
                diagnostics.append({'candidate_velocity_max_rad_s': vmax, 'error': str(exc)})

        if selected is not None:
            compact.update({
                'coulomb': float(selected['coulomb_Nm']),
                'static': float(selected['static_friction_amplitude_Nm']),
                'viscous': float(selected['viscous_Nm_per_rad_s']),
                'stribeck_velocity': float(selected['stribeck_velocity_rad_s']),
                'velocity_min': min(float(p['target_speed_rad_s']) for p in selected_pairs),
                'velocity_max': float(selected['candidate_velocity_max_rad_s']),
                'selected_model': 'stribeck',
            })
            details.update({
                'coulomb_friction_Nm': compact['coulomb'],
                'static_friction_amplitude_Nm': compact['static'],
                'viscous_friction_Nm_per_rad_s': compact['viscous'],
                'stribeck_velocity_rad_s': compact['stribeck_velocity'],
                'validated_velocity_min_rad_s': compact['velocity_min'],
                'validated_velocity_max_rad_s': compact['velocity_max'],
                'odd_component_fit_rmse_Nm': float(selected['fit_rmse_Nm']),
                'cross_repeat_validation_rmse_Nm': float(selected['stribeck_cross_repeat_rmse_Nm']),
                'cross_repeat_validation_max_rmse_Nm': float(
                    selected['stribeck_cross_repeat_max_rmse_Nm']),
                'quality': selected['fit_quality'],
            })
            selected_ids = {id(item) for item in selected_pairs}
            bias, even_spread = _robust_location_scale(np.asarray([
                float(item['even_residual_median_Nm']) for item in selected_pairs]))
            compact['bias'] = bias
            details['torque_bias_diagnostic_Nm'] = bias
            details['even_residual_mad_sigma_Nm'] = even_spread
            for item in paired:
                item['used_for_fit'] = id(item) in selected_ids
                v = float(item['matched_speed_median_rad_s'])
                predicted = float(stribeck(
                    np.asarray([v]), compact['coulomb'], compact['static'],
                    compact['viscous'], compact['stribeck_velocity'],
                    self.friction_smoothing_velocity,
                )[0])
                item['fitted_odd_friction_Nm'] = predicted
                item['odd_fit_error_Nm'] = float(item['odd_friction_median_Nm']) - predicted
        else:
            compact.update({
                'static': float(compact['coulomb']),
                'stribeck_velocity': 0.05,
                'selected_model': 'coulomb_fallback',
            })
            details.update({
                'static_friction_amplitude_Nm': compact['static'],
                'stribeck_velocity_rad_s': compact['stribeck_velocity'],
            })
            details['warnings'].append(
                'Stribeck peak not validated against held-out repeats; using Fc+Bdq (Fs=Fc)'
            )

        details['model_selected'] = compact['selected_model']
        details['stribeck_velocity_range_candidates'] = diagnostics
        return details, compact

    def _write_calibration_yaml(
        self,
        path: Path,
        calibration: List[Dict[str, float]],
        quality: List[str],
    ) -> None:
        """Write a controller-compatible schema-v4 friction calibration."""
        def array(key: str) -> str:
            return '[' + ', '.join(f"{float(item[key]):.10g}" for item in calibration) + ']'

        generated_at_unix_s = time.time()
        generated_at_utc = datetime.fromtimestamp(
            generated_at_unix_s, tz=timezone.utc
        ).isoformat().replace('+00:00', 'Z')
        lines = [
            '# Generated by dynamics_identification_experiment.py',
            '# Franka M(q), Coriolis and gravity remain unchanged.',
            '# Strictly odd model: tau_f=[Fc+(Fs-Fc)*exp(-(dq/vs)^2)]*tanh(dq/epsilon)+B*dq',
            '# Fs=Fc is the no-Stribeck-peak fallback if held-out validation fails.',
            '# Single-start-pose sweep; check performance away from the reference pose.',
            '# Even residual/bias is diagnostic only and is never applied as friction.',
            'friction_calibration:',
            '  schema_version: 4',
            f'  generated_at_unix_s: {generated_at_unix_s:.6f}',
            f'  generated_at_utc: "{generated_at_utc}"',
            '  recommended_max_age_hours: 24.0',
            '  model: odd_stribeck_viscous_tanh',
            '  joint_names: [' + ', '.join(self.joint_names) + ']',
            f'  smoothing_velocity_rad_s: {self.friction_smoothing_velocity:.10g}',
            '  calibration_scope: single_start_pose_sequential_joint_sweeps',
            '  reference_joint_position_rad: [' + ', '.join(
                f'{float(x):.10g}' for x in self.start_q
            ) + ']',
            f'  torque_bias_diagnostic_Nm: {array("bias")}',
            f'  coulomb_Nm: {array("coulomb")}',
            f'  viscous_Nm_per_rad_s: {array("viscous")}',
            f'  static_friction_amplitude_Nm: {array("static")}',
            f'  stribeck_velocity_rad_s: {array("stribeck_velocity")}',
            '  model_selected_by_joint: [' + ', '.join(
                str(item['selected_model']) for item in calibration
            ) + ']',
            f'  validated_velocity_min_rad_s: {array("velocity_min")}',
            f'  validated_velocity_max_rad_s: {array("velocity_max")}',
            '  quality: [' + ', '.join(quality) + ']',
            '  recommended_enable: [' + ', '.join(
                'true' if q != 'poor' else 'false' for q in quality
            ) + ']',
            '  apply_torque_bias: false',
        ]
        path.write_text('\n'.join(lines) + '\n', encoding='utf-8')

    def _save_and_fit(self):
        if len(self.records) < 20:
            raise RuntimeError('not enough observations were collected')

        self.output_directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        csv_path = self.output_directory / f'friction_identification_{stamp}.csv'
        json_path = self.output_directory / f'friction_identification_{stamp}.json'
        calibration_path = self.output_directory / f'friction_calibration_{stamp}.yaml'

        header = ['time_s']
        for block in [
            'q', 'dq', 'ddq', 'tau_measured', 'tau_inertia', 'coriolis', 'gravity',
            'tau_model', 'residual', 'command_velocity'
        ]:
            header.extend([f'{block}_{i + 1}' for i in range(N)])
        header.extend(['fit_joint', 'fit_repeat', 'fit_active', 'target_speed_rad_s'])
        header.extend([f'M_{r + 1}_{c + 1}' for c in range(N) for r in range(N)])

        t0 = float(self.records[0]['t'])
        with csv_path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for record in self.records:
                row = [float(record['t']) - t0]
                for block in [
                    'q', 'dq', 'ddq', 'tau_measured', 'tau_inertia', 'coriolis',
                    'gravity', 'tau_model', 'residual', 'command_velocity'
                ]:
                    row.extend(np.asarray(record[block]).tolist())
                row.extend([
                    int(record['fit_joint']) + 1 if int(record['fit_joint']) >= 0 else 0,
                    int(record['fit_repeat']) + 1 if int(record['fit_repeat']) >= 0 else 0,
                    int(bool(record['fit_active'])),
                    float(record['target_speed']),
                ])
                row.extend(np.asarray(record['mass']).reshape(-1, order='F').tolist())
                writer.writerow(row)

        q = np.stack([np.asarray(r['q']) for r in self.records])
        dq = np.stack([np.asarray(r['dq']) for r in self.records])
        ddq = np.stack([np.asarray(r['ddq']) for r in self.records])
        tau_measured = np.stack([np.asarray(r['tau_measured']) for r in self.records])
        tau_model = np.stack([np.asarray(r['tau_model']) for r in self.records])
        baseline_error = tau_measured - tau_model
        fit_joint = np.asarray([int(r['fit_joint']) for r in self.records], dtype=int)
        fit_repeat = np.asarray([int(r['fit_repeat']) for r in self.records], dtype=int)
        fit_active = np.asarray([bool(r['fit_active']) for r in self.records], dtype=bool)
        target_speed = np.asarray([float(r['target_speed']) for r in self.records])

        summary: Dict[str, object] = {
            'note': (
                'Franka M(q), Coriolis and gravity are kept fixed. Friction is identified from '
                'position-matched +/- velocity pairs using only the odd torque component. The '
                'even residual is diagnostic and is never applied as friction.'
            ),
            'model': (
                'tau_f=[Fc+(Fs-Fc)*exp(-(dq/vs)^2)]*tanh(dq/epsilon)+B*dq; '
                'Fs>=Fc>=0, B>=0; epsilon fixed; position-matched odd residual fit; '
                'held-out-repeat model selection versus Coulomb+viscous baseline'
            ),
            'experiment': 'single_start_pose_sequential_joint_stribeck_sweep',
            'reference_joint_position_rad': self.start_q.tolist(),
            'single_start_pose_only': True,
            'estimated_excitation_duration_s': self.friction_schedule_ends[-1],
            'motion_mode': self.motion_mode,
            'run_motion': self.run_motion,
            'aborted': self.abort_reason is not None,
            'abort_reason': self.abort_reason,
            'samples': int(len(self.records)),
            'duration_s': float(float(self.records[-1]['t']) - float(self.records[0]['t'])),
            'velocity_levels_rad_s': self.friction_velocity_levels.tolist(),
            'repeats': self.friction_repeats,
            'ramp_time_s': self.friction_ramp_time,
            'hold_time_min_s': self.friction_hold_time,
            'hold_time_max_s': self.friction_max_hold_time,
            'min_plateau_travel_rad': self.friction_min_plateau_travel_rad,
            'speed_tracking_tolerance_ratio': self.friction_speed_tolerance_ratio,
            'pause_time_s': self.friction_pause_time,
            'plateau_trim_time_s': self.friction_plateau_trim_time,
            'min_fit_velocity_rad_s': self.friction_min_fit_velocity,
            'max_fit_acceleration_rad_s2': self.friction_max_fit_acceleration,
            'weight_floor_Nm': self.friction_weight_floor,
            'pair_grid_points': self.friction_pair_grid_points,
            'min_pair_samples': self.friction_min_pair_samples,
            'smoothing_velocity_rad_s': self.friction_smoothing_velocity,
            'joints': [],
        }

        compact_calibration: List[Dict[str, float]] = []
        compact_quality: List[str] = []
        all_joints_calibrated = True

        self.get_logger().info(
            'Per-joint strictly odd Stribeck versus Coulomb baseline, matched +/- sweeps:'
        )
        for j in range(N):
            if self.motion_mode == 'joint' and self.run_motion:
                mask = fit_active & (fit_joint == j)
            else:
                mask = np.zeros(len(self.records), dtype=bool)

            mask &= np.abs(dq[:, j]) >= self.friction_min_fit_velocity
            mask &= np.abs(ddq[:, j]) <= self.friction_max_fit_acceleration
            if self.motion_mode == 'joint' and self.run_motion:
                mask &= np.abs(dq[:, j] - target_speed) <= np.maximum(
                    0.003, self.friction_speed_tolerance_ratio * np.abs(target_speed)
                )

            v = dq[mask, j]
            residual = baseline_error[mask, j]
            joint_result: Dict[str, object] = {
                'joint': j + 1,
                'q_range_rad': float(np.ptp(q[:, j])),
                'dq_peak_rad_s': float(np.max(np.abs(dq[:, j]))),
                'fit_status': 'ok',
            }

            plateau_stats: List[Dict[str, object]] = []
            if self.motion_mode == 'joint' and self.run_motion:
                for repeat_index in range(self.friction_repeats):
                    for speed in self.friction_velocity_levels:
                        for sign in (1.0, -1.0):
                            signed_speed = float(sign * speed)
                            level_mask = (
                                mask
                                & (fit_repeat == repeat_index)
                                & np.isclose(target_speed, signed_speed, atol=1e-8)
                            )
                            if not np.any(level_mask):
                                continue
                            level_v = dq[level_mask, j]
                            level_residual = baseline_error[level_mask, j]
                            residual_median, residual_mad = _robust_location_scale(
                                level_residual
                            )
                            velocity_median, velocity_mad = _robust_location_scale(level_v)
                            plateau_stats.append({
                                'repeat': repeat_index + 1,
                                'target_velocity_rad_s': signed_speed,
                                'samples': int(np.count_nonzero(level_mask)),
                                'measured_velocity_mean_rad_s': float(np.mean(level_v)),
                                'measured_velocity_std_rad_s': float(np.std(level_v)),
                                'measured_velocity_median_rad_s': velocity_median,
                                'measured_velocity_mad_sigma_rad_s': velocity_mad,
                                'franka_residual_mean_Nm': float(np.mean(level_residual)),
                                'franka_residual_std_Nm': float(np.std(level_residual)),
                                'franka_residual_median_Nm': residual_median,
                                'franka_residual_mad_sigma_Nm': residual_mad,
                            })

            try:
                if v.size < 40:
                    raise RuntimeError(f'only {v.size} usable plateau samples')
                paired = self._build_position_matched_pairs(
                    j, q, dq, baseline_error, fit_repeat, target_speed, mask
                )
                fit, compact = self._fit_joint_friction_from_pairs(paired)
                fit, compact = self._fit_stribeck_or_fallback(paired, fit, compact)
                joint_result.update(fit)

                # Keep conventional plateau statistics for inspection. The fit itself
                # uses the position-matched odd observations stored above.
                for item in plateau_stats:
                    measured_velocity = float(item['measured_velocity_median_rad_s'])
                    item['friction_prediction_Nm'] = float(
                        stribeck(
                            np.asarray([measured_velocity]),
                            compact['coulomb'], compact['static'], compact['viscous'],
                            compact['stribeck_velocity'], self.friction_smoothing_velocity,
                        )[0]
                    )
                joint_result['plateau_statistics'] = plateau_stats

                compact_calibration.append(compact)
                compact_quality.append(str(fit['quality']))

                warning_suffix = (
                    '' if not fit['warnings'] else '; warnings: ' + '; '.join(fit['warnings'])
                )
                cv = fit['cross_repeat_validation_rmse_Nm']
                cv_text = 'n/a' if cv is None else f'{cv:.3f}'
                self.get_logger().info(
                    f"J{j + 1}: Fc={fit['coulomb_friction_Nm']:.3f} Nm, "
                    f"Fs={fit['static_friction_amplitude_Nm']:.3f} Nm, "
                    f"vs={fit['stribeck_velocity_rad_s']:.3f} rad/s, "
                    f"B={fit['viscous_friction_Nm_per_rad_s']:.3f} Nm/(rad/s), "
                    f"selected={fit['model_selected']}, "
                    f"odd RMSE={fit['odd_component_fit_rmse_Nm']:.3f} Nm, "
                    f"cross-repeat={cv_text} Nm, "
                    f"validated<= {fit['validated_velocity_max_rad_s']:.3f} rad/s, "
                    f"quality={fit['quality']}{warning_suffix}"
                )
            except Exception as exc:  # noqa: BLE001
                all_joints_calibrated = False
                joint_result.update({
                    'fit_status': 'insufficient_data',
                    'fit_error': str(exc),
                    'samples_candidate': int(v.size),
                    'plateau_statistics': plateau_stats,
                })
                self.get_logger().warn(f'J{j + 1}: friction fit skipped: {exc}')

            summary['joints'].append(joint_result)

        with json_path.open('w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2)

        if (self.abort_reason is None and self.run_motion and
                self.motion_mode == 'joint' and
                all_joints_calibrated and len(compact_calibration) == N and
                any(q != 'poor' for q in compact_quality)):
            self._write_calibration_yaml(
                calibration_path, compact_calibration, compact_quality
            )
            output_calibration_path = calibration_path
        else:
            output_calibration_path = None

        return csv_path, json_path, output_calibration_path


def main() -> None:
    rclpy.init()
    node = DynamicsIdentificationExperiment()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._publish_zero()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
