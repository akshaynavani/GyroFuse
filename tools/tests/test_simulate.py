import numpy as np
import pytest

from gyrofuse_tools.io import load_imu_csv, write_imu_csv
from gyrofuse_tools.sim import G, SimConfig, body_rates, rotation_zyx, simulate


def test_rotation_is_orthonormal():
    rng = np.random.default_rng(0)
    r, p, y = rng.uniform(-1, 1, (3, 50))
    R = rotation_zyx(r, p, y)
    eye = np.einsum("nij,nkj->nik", R, R)
    np.testing.assert_allclose(eye, np.broadcast_to(np.eye(3), eye.shape), atol=1e-12)


def test_body_rates_level_equal_euler_rates():
    p, q, r = body_rates(0.0, 0.0, 0.1, 0.2, 0.3)
    assert (p, q, r) == pytest.approx((0.1, 0.2, 0.3))


def test_static_sensor_reads_gravity():
    cfg = SimConfig(duration=2.0, still_seconds=5.0, accel_noise=0.0, gyro_noise=0.0,
                    gyro_bias=(0, 0, 0), gyro_bias_walk=0.0, bursts=[])
    df = simulate(cfg)
    np.testing.assert_allclose(df["az"], G, atol=1e-9)
    np.testing.assert_allclose(df[["ax", "ay", "gx", "gy", "gz"]].to_numpy(), 0.0, atol=1e-9)


def test_accel_tilt_matches_reference_without_noise():
    cfg = SimConfig(duration=20.0, accel_noise=0.0, gyro_noise=0.0, bursts=[])
    df = simulate(cfg)
    roll = np.arctan2(df["ay"], df["az"])
    pitch = np.arctan2(-df["ax"], np.hypot(df["ay"], df["az"]))
    np.testing.assert_allclose(roll, df["roll_ref"], atol=1e-9)
    np.testing.assert_allclose(pitch, df["pitch_ref"], atol=1e-9)


def test_gyro_integrates_to_reference_roll():
    cfg = SimConfig(duration=10.0, rate_hz=1000.0, pitch_amp_deg=0.0, yaw_rate_deg=0.0,
                    accel_noise=0.0, gyro_noise=0.0, gyro_bias=(0, 0, 0), gyro_bias_walk=0.0, bursts=[])
    df = simulate(cfg)
    integrated = np.cumsum(df["gx"]) / cfg.rate_hz
    assert np.max(np.abs(integrated - df["roll_ref"])) < np.deg2rad(0.5)


def test_csv_roundtrip(tmp_path):
    df = simulate(SimConfig(duration=1.0, rate_hz=50.0))
    path = tmp_path / "sim.csv"
    write_imu_csv(df, path)
    back = load_imu_csv(path)
    assert list(back.columns) == list(df.columns)
    np.testing.assert_allclose(back.to_numpy(), df.to_numpy(), atol=1e-6)
