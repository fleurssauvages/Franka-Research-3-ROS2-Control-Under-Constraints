"""Pure-NumPy/SciPy friction-fitting routines (also testable without ROS).

All identification uses strictly odd, smoothed friction. 'Static amplitude' is
NOT a breakaway torque at rest: tanh(v / epsilon) makes tau(0) = 0.
"""

import numpy as np
from scipy.optimize import least_squares


def coulomb_viscous(velocity, fc, b, epsilon):
    v = np.asarray(velocity, dtype=float)
    return fc * np.tanh(v / epsilon) + b * v


def stribeck(velocity, fc, fs, b, vs, epsilon):
    v = np.asarray(velocity, dtype=float)
    return (fc + (fs - fc) * np.exp(-(v / vs) ** 2)) * np.tanh(v / epsilon) + b * v


def _arrays(pairs, weight_floor):
    speed = np.asarray([float(p['matched_speed_median_rad_s']) for p in pairs])
    torque = np.asarray([float(p['odd_friction_median_Nm']) for p in pairs])
    sigma = np.asarray([
        max(weight_floor, float(p['fit_uncertainty_Nm'])) for p in pairs
    ])
    if (speed.size < 10 or np.any(speed <= 0) or
            not np.all(np.isfinite(speed + torque + sigma))):
        raise ValueError('insufficient or invalid position-matched speed/torque data')
    return speed, torque, sigma


def _fit_coulomb(pairs, epsilon, weight_floor):
    velocity, torque, sigma = _arrays(pairs, weight_floor)
    # Full robust least squares rather than estimating friction from one noisy plateau.
    starts = ((0.15, 0.5), (0.4, 0.1), (0.05, 2.0))
    solutions = []
    for start in starts:
        result = least_squares(
            lambda x: (coulomb_viscous(velocity, x[0], x[1], epsilon) - torque) / sigma,
            start, bounds=([0.0, 0.0], [3.0, 10.0]),
            loss='soft_l1', f_scale=1.5, max_nfev=2500,
        )
        if result.success:
            solutions.append(result)
    if not solutions:
        raise RuntimeError('Coulomb baseline robust fit failed')
    best = min(solutions, key=lambda x: x.cost)
    return tuple(float(x) for x in best.x)


def _fit_stribeck(pairs, epsilon, weight_floor, baseline):
    velocity, torque, sigma = _arrays(pairs, weight_floor)
    fc0, b0 = baseline
    solutions = []
    for vs0 in (0.018, 0.04, 0.08, 0.12):
        for excess0 in (0.05, 0.3):
            start = [min(fc0, 2.5), excess0, min(b0, 8.0), vs0]
            result = least_squares(
                lambda x: (
                    stribeck(velocity, x[0], x[0] + x[1], x[2], x[3], epsilon)
                    - torque
                ) / sigma,
                start,
                bounds=([0.0, 0.0, 0.0, 0.006], [3.0, 1.5, 10.0, 0.16]),
                loss='soft_l1', f_scale=1.5, max_nfev=2500,
            )
            if result.success:
                solutions.append(result)
    if not solutions:
        raise RuntimeError('Stribeck robust fit failed')
    best = min(solutions, key=lambda x: x.cost)
    fc, excess, b, vs = (float(x) for x in best.x)
    return (fc, fc + excess, b, vs)


def _quality(rmse, median_noise, cross_rmse, cross_max):
    if (rmse <= 0.04 and median_noise <= 0.08 and
            cross_rmse <= 0.05 and cross_max <= 0.08):
        return 'good'
    if (rmse <= 0.08 and median_noise <= 0.15 and
            cross_rmse <= 0.09 and cross_max <= 0.13):
        return 'usable'
    return 'poor'


def evaluate_stribeck(pairs, epsilon, weight_floor=0.02):
    """Fit + repeat-held-out CV, compare to the Coulomb baseline on the SAME pairs.

    Returns a diagnostics dict; it is the caller's responsibility to ensure
    the input contains sufficiently independent speed levels and repeats.
    """
    velocity, truth, sigma = _arrays(pairs, weight_floor)
    base = _fit_coulomb(pairs, epsilon, weight_floor)
    fitted = _fit_stribeck(pairs, epsilon, weight_floor, base)
    pred = stribeck(velocity, *fitted, epsilon)
    rmse = float(np.sqrt(np.mean((truth - pred) ** 2)))
    median_noise = float(np.median(sigma))

    repeats = sorted({int(p['repeat']) for p in pairs})
    base_cross, stribeck_cross = [], []
    for held in repeats:
        train = [p for p in pairs if int(p['repeat']) != held]
        test = [p for p in pairs if int(p['repeat']) == held]
        if len(train) < 10 or len(test) < 4:
            continue
        base_fit = _fit_coulomb(train, epsilon, weight_floor)
        str_fit = _fit_stribeck(train, epsilon, weight_floor, base_fit)
        speeds = np.asarray([float(p['matched_speed_median_rad_s']) for p in test])
        torque = np.asarray([float(p['odd_friction_median_Nm']) for p in test])
        base_cross.append(float(np.sqrt(np.mean(
            (torque - coulomb_viscous(speeds, *base_fit, epsilon)) ** 2))))
        stribeck_cross.append(float(np.sqrt(np.mean(
            (torque - stribeck(speeds, *str_fit, epsilon)) ** 2))))
    if len(base_cross) < 3 or len(stribeck_cross) < 3:
        raise RuntimeError('at least three hold-out repeats required')
    base_cv = float(np.mean(base_cross))
    str_cv = float(np.mean(stribeck_cross))
    minimum_gain = max(0.005, 0.10 * base_cv)
    good_identifiability = (
        fitted[1] - fitted[0] >= 0.02 and
        0.008 < fitted[3] < 0.145
    )
    quality = _quality(rmse, median_noise, str_cv, max(stribeck_cross))
    select = (str_cv + minimum_gain < base_cv and good_identifiability and
              quality != 'poor')
    return {
        'selected': bool(select),
        'coulomb_Nm': fitted[0],
        'static_friction_amplitude_Nm': fitted[1],
        'viscous_Nm_per_rad_s': fitted[2],
        'stribeck_velocity_rad_s': fitted[3],
        'fit_rmse_Nm': rmse,
        'baseline_cross_repeat_rmse_Nm': base_cv,
        'stribeck_cross_repeat_rmse_Nm': str_cv,
        'stribeck_cross_repeat_max_rmse_Nm': float(max(stribeck_cross)),
        'min_required_cv_improvement_Nm': minimum_gain,
        'fit_quality': quality,
        'has_identifiable_peak': bool(good_identifiability),
    }
