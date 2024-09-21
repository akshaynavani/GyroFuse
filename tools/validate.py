#!/usr/bin/env python3
"""Run every GyroFuse filter over a dataset and score it against ground truth.

The dataset must contain roll_ref/pitch_ref columns (e.g. from simulate.py, or a
capture with an external reference). Exits non-zero if any filter exceeds the
RMSE threshold, so it doubles as a regression check.

Example:
    python tools/validate.py data/sim.csv --binary build/gyrofuse --calibrate 2 --plot out/validation.png
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

from gyrofuse_tools.io import has_reference, load_estimates, load_imu_csv
from gyrofuse_tools.metrics import summarize

FILTERS = ("complementary", "kalman")


def run_filter(binary: Path, dataset: Path, filt: str, out: Path, calibrate: float) -> None:
    cmd = [str(binary), "--input", str(dataset), "--filter", filt, "--output", str(out)]
    if calibrate > 0:
        cmd += ["--calibrate", str(calibrate)]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        sys.exit(f"{' '.join(cmd)} failed:\n{res.stderr}")


def accel_only_stats(raw, skip: int) -> dict:
    """Baseline: attitude straight from the accelerometer, no fusion."""
    import pandas as pd

    df = pd.DataFrame(
        {
            "roll": np.arctan2(raw["ay"], raw["az"]),
            "pitch": np.arctan2(-raw["ax"], np.hypot(raw["ay"], raw["az"])),
            "roll_ref": raw["roll_ref"],
            "pitch_ref": raw["pitch_ref"],
        }
    )
    return summarize(df, skip)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dataset", type=Path)
    ap.add_argument("--binary", type=Path, default=Path("build/gyrofuse"))
    ap.add_argument("--out-dir", type=Path, default=Path("out"))
    ap.add_argument("--calibrate", type=float, default=0.0, help="seconds of still data for gyro bias")
    ap.add_argument("--skip", type=float, default=1.0, help="seconds of initial transient to ignore")
    ap.add_argument("--max-rmse", type=float, default=3.0, help="roll/pitch RMSE limit in degrees")
    ap.add_argument("--json", type=Path, help="also write the report as JSON")
    ap.add_argument("--plot", type=Path, help="save a comparison plot")
    args = ap.parse_args()

    raw = load_imu_csv(args.dataset)
    if not has_reference(raw):
        sys.exit("dataset has no roll_ref/pitch_ref columns; nothing to validate against")
    if not args.binary.exists():
        sys.exit(f"{args.binary} not found; build with: cmake -B build && cmake --build build")

    rate = 1.0 / np.median(np.diff(raw["t"]))
    skip = int(args.skip * rate)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    report = {"dataset": str(args.dataset), "samples": len(raw), "rate_hz": round(float(rate), 2)}
    report["accel_only"] = accel_only_stats(raw, skip)
    outputs = []
    for filt in FILTERS:
        out = args.out_dir / f"{filt}.csv"
        run_filter(args.binary, args.dataset, filt, out, args.calibrate)
        report[filt] = summarize(load_estimates(out), skip)
        outputs.append(out)

    print(f"{'method':<15}{'axis':<7}{'RMSE':>8}{'P95':>8}{'max':>8}{'mean':>8}   [deg]")
    failed = False
    for method in ("accel_only",) + FILTERS:
        for axis, st in report[method].items():
            print(f"{method:<15}{axis:<7}{st['rmse_deg']:8.3f}{st['p95_deg']:8.3f}"
                  f"{st['max_abs_deg']:8.3f}{st['mean_deg']:8.3f}")
            if method in FILTERS and axis in ("roll", "pitch") and st["rmse_deg"] > args.max_rmse:
                failed = True

    if args.json:
        args.json.write_text(json.dumps(report, indent=2))
    if args.plot:
        cmd = [sys.executable, str(Path(__file__).with_name("visualize.py")), *map(str, outputs),
               "--raw", str(args.dataset), "--save", str(args.plot)]
        subprocess.run(cmd, check=False)

    if failed:
        print(f"FAIL: roll/pitch RMSE above {args.max_rmse} deg")
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
