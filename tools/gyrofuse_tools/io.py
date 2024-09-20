"""CSV I/O matching the C++ reader/writer formats."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

IMU_COLUMNS = ["t", "ax", "ay", "az", "gx", "gy", "gz"]
REF_COLUMNS = ["roll_ref", "pitch_ref", "yaw_ref"]
EST_COLUMNS = ["t", "roll", "pitch", "yaw", "bias_x", "bias_y", "bias_z"]


def load_imu_csv(path: str | Path) -> pd.DataFrame:
    """Load a raw IMU dataset (t, accel m/s^2, gyro rad/s, optional ref angles rad)."""
    df = pd.read_csv(path, comment="#")
    df.columns = [c.strip().lower() for c in df.columns]
    missing = [c for c in IMU_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")
    return df


def has_reference(df: pd.DataFrame) -> bool:
    return "roll_ref" in df.columns and "pitch_ref" in df.columns


def write_imu_csv(df: pd.DataFrame, path: str | Path) -> None:
    cols = IMU_COLUMNS + [c for c in REF_COLUMNS if c in df.columns]
    df.to_csv(path, columns=cols, index=False, float_format="%.6f")


def load_estimates(path: str | Path, degrees: bool = False) -> pd.DataFrame:
    """Load filter output written by the gyrofuse CLI. Angles are returned in radians."""
    df = pd.read_csv(path)
    missing = [c for c in EST_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")
    if degrees:
        angle_cols = ["roll", "pitch", "yaw"] + [c for c in REF_COLUMNS if c in df.columns]
        df[angle_cols] = np.deg2rad(df[angle_cols])
    return df
