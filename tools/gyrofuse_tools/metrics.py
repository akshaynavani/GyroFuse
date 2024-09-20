"""Error metrics for comparing attitude estimates against ground truth."""

from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np


def wrap(angle: np.ndarray) -> np.ndarray:
    """Wrap angles (rad) into (-pi, pi]."""
    a = np.mod(np.asarray(angle, dtype=float) + np.pi, 2.0 * np.pi)
    a = np.where(a <= 0.0, a + 2.0 * np.pi, a)
    return a - np.pi


def angle_error(est: np.ndarray, ref: np.ndarray) -> np.ndarray:
    return wrap(np.asarray(est) - np.asarray(ref))


def rmse(est: np.ndarray, ref: np.ndarray) -> float:
    err = angle_error(est, ref)
    return float(np.sqrt(np.mean(err**2))) if err.size else float("nan")


@dataclass
class AxisStats:
    rmse_deg: float
    max_abs_deg: float
    mean_deg: float  # signed bias of the error
    p95_deg: float


def axis_stats(est: np.ndarray, ref: np.ndarray, skip: int = 0) -> AxisStats:
    """Error statistics in degrees, optionally skipping the initial transient."""
    err = np.rad2deg(angle_error(est, ref))[skip:]
    if err.size == 0:
        nan = float("nan")
        return AxisStats(nan, nan, nan, nan)
    return AxisStats(
        rmse_deg=float(np.sqrt(np.mean(err**2))),
        max_abs_deg=float(np.max(np.abs(err))),
        mean_deg=float(np.mean(err)),
        p95_deg=float(np.percentile(np.abs(err), 95)),
    )


def summarize(est_df, skip: int = 0) -> dict:
    """Per-axis stats for an estimates DataFrame that carries *_ref columns."""
    out = {}
    for axis in ("roll", "pitch", "yaw"):
        ref_col = f"{axis}_ref"
        if ref_col in est_df.columns:
            out[axis] = asdict(axis_stats(est_df[axis].to_numpy(), est_df[ref_col].to_numpy(), skip))
    return out
