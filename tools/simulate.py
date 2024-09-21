#!/usr/bin/env python3
"""Generate a synthetic IMU dataset with ground-truth attitude.

Example:
    python tools/simulate.py --out data/sim.csv --duration 60 --rate 200
"""

from __future__ import annotations

import argparse
from pathlib import Path

from gyrofuse_tools.io import write_imu_csv
from gyrofuse_tools.sim import SimConfig, simulate


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=Path("data/sim.csv"))
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--rate", type=float, default=200.0, help="sample rate in Hz")
    ap.add_argument("--roll-amp", type=float, default=30.0, help="deg")
    ap.add_argument("--pitch-amp", type=float, default=20.0, help="deg")
    ap.add_argument("--yaw-rate", type=float, default=10.0, help="deg/s")
    ap.add_argument("--gyro-noise", type=float, default=0.005, help="rad/s std")
    ap.add_argument("--accel-noise", type=float, default=0.05, help="m/s^2 std")
    ap.add_argument("--bias", type=float, nargs=3, default=(0.02, -0.015, 0.01), help="rad/s")
    ap.add_argument("--no-bursts", action="store_true", help="disable linear acceleration bursts")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    cfg = SimConfig(
        duration=args.duration,
        rate_hz=args.rate,
        roll_amp_deg=args.roll_amp,
        pitch_amp_deg=args.pitch_amp,
        yaw_rate_deg=args.yaw_rate,
        gyro_noise=args.gyro_noise,
        accel_noise=args.accel_noise,
        gyro_bias=tuple(args.bias),
        seed=args.seed,
    )
    if args.no_bursts:
        cfg.bursts = []

    df = simulate(cfg)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_imu_csv(df, args.out)
    print(f"wrote {len(df)} samples ({cfg.duration:.1f}s @ {cfg.rate_hz:.0f} Hz) to {args.out}")


if __name__ == "__main__":
    main()
