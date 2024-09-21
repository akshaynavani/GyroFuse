#!/usr/bin/env python3
"""Plot one or more GyroFuse estimate files against each other and ground truth.

Example:
    python tools/visualize.py out/kalman.csv out/complementary.csv --raw data/sim.csv --save out/plot.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from gyrofuse_tools.io import load_estimates, load_imu_csv
from gyrofuse_tools.metrics import angle_error


def accel_tilt(raw):
    roll = np.arctan2(raw["ay"], raw["az"])
    pitch = np.arctan2(-raw["ax"], np.hypot(raw["ay"], raw["az"]))
    return roll, pitch


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("estimates", nargs="+", type=Path, help="CSV files written by the gyrofuse CLI")
    ap.add_argument("--raw", type=Path, help="raw IMU CSV to overlay accel-only tilt")
    ap.add_argument("--degrees-input", action="store_true", help="estimates were written with --degrees")
    ap.add_argument("--save", type=Path, help="write the figure instead of showing it")
    args = ap.parse_args()

    runs = {p.stem: load_estimates(p, degrees=args.degrees_input) for p in args.estimates}
    ref = next((df for df in runs.values() if "roll_ref" in df.columns), None)

    n_rows = 4 if ref is not None else 3
    fig, axes = plt.subplots(n_rows, 1, figsize=(12, 2.8 * n_rows), sharex=True)
    axis_names = ("roll", "pitch", "yaw")

    if args.raw:
        raw = load_imu_csv(args.raw)
        r, p = accel_tilt(raw)
        axes[0].plot(raw["t"], np.rad2deg(r), color="0.75", lw=0.6, label="accel only")
        axes[1].plot(raw["t"], np.rad2deg(p), color="0.75", lw=0.6, label="accel only")

    for i, axis in enumerate(axis_names):
        ax = axes[i]
        if ref is not None:
            ax.plot(ref["t"], np.rad2deg(ref[f"{axis}_ref"]), "k--", lw=1.2, label="reference")
        for name, df in runs.items():
            ax.plot(df["t"], np.rad2deg(df[axis]), lw=1.0, label=name)
        ax.set_ylabel(f"{axis} [deg]")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

    if ref is not None:
        ax = axes[3]
        for name, df in runs.items():
            if "roll_ref" not in df.columns:
                continue
            for axis, style in (("roll", "-"), ("pitch", ":")):
                err = np.rad2deg(angle_error(df[axis], df[f"{axis}_ref"]))
                ax.plot(df["t"], err, style, lw=0.9, label=f"{name} {axis}")
        ax.set_ylabel("error [deg]")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8, ncol=2)

    axes[-1].set_xlabel("time [s]")
    fig.suptitle("GyroFuse attitude estimates")
    fig.tight_layout()

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=130)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
