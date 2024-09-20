"""Synthetic IMU generator with exact ground-truth attitude.

The true attitude follows smooth sinusoids (ZYX Euler). Body rates are derived
from the Euler-rate kinematics and the accelerometer measures specific force
f = R^T (a_world + g * z_up), so a level, still sensor reads +g on z — the same
convention the C++ filters assume.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import pandas as pd

G = 9.80665


@dataclass
class SimConfig:
    duration: float = 60.0
    rate_hz: float = 200.0
    roll_amp_deg: float = 30.0
    pitch_amp_deg: float = 20.0
    yaw_rate_deg: float = 10.0
    roll_freq_hz: float = 0.20
    pitch_freq_hz: float = 0.13
    accel_noise: float = 0.05            # m/s^2 std
    gyro_noise: float = 0.005            # rad/s std
    gyro_bias: tuple = (0.02, -0.015, 0.01)  # rad/s, constant
    gyro_bias_walk: float = 0.0005       # rad/s/sqrt(s)
    still_seconds: float = 3.0           # leading stationary window for calibration
    bursts: list = field(default_factory=lambda: [(20.0, 1.0, 4.0), (40.0, 0.5, 6.0)])
    # each burst: (start_s, duration_s, peak linear accel m/s^2 along world x)
    seed: int = 7


def rotation_zyx(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    """Body->world rotation matrices, shape (N, 3, 3)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.empty((roll.size, 3, 3))
    R[:, 0, 0] = cy * cp
    R[:, 0, 1] = cy * sp * sr - sy * cr
    R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp
    R[:, 1, 1] = sy * sp * sr + cy * cr
    R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp
    R[:, 2, 1] = cp * sr
    R[:, 2, 2] = cp * cr
    return R


def body_rates(roll, pitch, roll_dot, pitch_dot, yaw_dot):
    """Inverse Euler kinematics: Euler rates -> body angular velocity (p, q, r)."""
    p = roll_dot - np.sin(pitch) * yaw_dot
    q = np.cos(roll) * pitch_dot + np.sin(roll) * np.cos(pitch) * yaw_dot
    r = -np.sin(roll) * pitch_dot + np.cos(roll) * np.cos(pitch) * yaw_dot
    return p, q, r


def _ramp(t: np.ndarray, start: float) -> np.ndarray:
    """0 before `start`, smooth 0->1 over one second after it."""
    x = np.clip(t - start, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * x)


def simulate(cfg: SimConfig = SimConfig()) -> pd.DataFrame:
    rng = np.random.default_rng(cfg.seed)
    dt = 1.0 / cfg.rate_hz
    t = np.arange(0.0, cfg.duration, dt)

    env = _ramp(t, cfg.still_seconds)
    d_env = np.gradient(env, dt)
    wr, wp = 2 * np.pi * cfg.roll_freq_hz, 2 * np.pi * cfg.pitch_freq_hz
    tm = t - cfg.still_seconds
    ra, pa = np.deg2rad(cfg.roll_amp_deg), np.deg2rad(cfg.pitch_amp_deg)

    roll = ra * env * np.sin(wr * tm)
    pitch = pa * env * np.sin(wp * tm)
    yaw_rate = np.deg2rad(cfg.yaw_rate_deg) * env
    yaw = np.cumsum(yaw_rate) * dt

    roll_dot = ra * (d_env * np.sin(wr * tm) + env * wr * np.cos(wr * tm))
    pitch_dot = pa * (d_env * np.sin(wp * tm) + env * wp * np.cos(wp * tm))
    p, q, r = body_rates(roll, pitch, roll_dot, pitch_dot, yaw_rate)

    # Linear acceleration bursts (world frame, half-sine profile).
    a_world = np.zeros((t.size, 3))
    for start, length, peak in cfg.bursts:
        mask = (t >= start) & (t < start + length)
        a_world[mask, 0] += peak * np.sin(np.pi * (t[mask] - start) / length)
    a_world[:, 2] += G

    R = rotation_zyx(roll, pitch, yaw)
    f_body = np.einsum("nji,nj->ni", R, a_world)  # R^T @ a

    bias = np.asarray(cfg.gyro_bias, dtype=float) + np.cumsum(
        rng.normal(0.0, cfg.gyro_bias_walk * np.sqrt(dt), (t.size, 3)), axis=0
    )
    gyro = np.column_stack([p, q, r]) + bias + rng.normal(0.0, cfg.gyro_noise, (t.size, 3))
    accel = f_body + rng.normal(0.0, cfg.accel_noise, (t.size, 3))

    def wrap(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    return pd.DataFrame(
        {
            "t": t,
            "ax": accel[:, 0], "ay": accel[:, 1], "az": accel[:, 2],
            "gx": gyro[:, 0], "gy": gyro[:, 1], "gz": gyro[:, 2],
            "roll_ref": wrap(roll), "pitch_ref": wrap(pitch), "yaw_ref": wrap(yaw),
        }
    )
