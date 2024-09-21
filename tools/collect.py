#!/usr/bin/env python3
"""Capture raw IMU data to a GyroFuse CSV.

Two backends:

  serial  A microcontroller streaming lines of "ax,ay,az,gx,gy,gz" (optionally
          prefixed by a timestamp in ms when --mcu-timestamps is set). Units are
          converted with --accel-unit / --gyro-unit.
  iio     A Linux IIO IMU read straight from sysfs (same as the C++ IioSource).

Optional reference angles can be appended via --ref-column-count when the MCU
also streams an external reference (e.g. a motion-capture or encoder angle).

Examples:
    python tools/collect.py serial --port /dev/ttyACM0 --baud 115200 --seconds 30 --out data/run1.csv
    python tools/collect.py iio --device auto --rate 200 --seconds 30 --out data/run2.csv
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

G = 9.80665
ACCEL_UNITS = {"g": G, "mps2": 1.0, "mg": G / 1000.0}
GYRO_UNITS = {"dps": math.pi / 180.0, "rads": 1.0, "mdps": math.pi / 180000.0}


def open_output(path: Path, with_ref: bool):
    path.parent.mkdir(parents=True, exist_ok=True)
    f = path.open("w", encoding="utf-8")
    header = "t,ax,ay,az,gx,gy,gz" + (",roll_ref,pitch_ref,yaw_ref" if with_ref else "")
    f.write(header + "\n")
    return f


def collect_serial(args) -> int:
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial is required: pip install -r tools/requirements.txt")

    ka, kg = ACCEL_UNITS[args.accel_unit], GYRO_UNITS[args.gyro_unit]
    n_expected = 6 + (1 if args.mcu_timestamps else 0) + (3 if args.with_ref else 0)
    count = 0
    with serial.Serial(args.port, args.baud, timeout=1.0) as port, open_output(args.out, args.with_ref) as out:
        port.reset_input_buffer()
        t_start = time.monotonic()
        mcu_t0 = None
        while time.monotonic() - t_start < args.seconds:
            line = port.readline().decode("ascii", errors="ignore").strip()
            if not line or line.startswith("#"):
                continue
            try:
                vals = [float(v) for v in line.split(",")]
            except ValueError:
                continue
            if len(vals) != n_expected:
                continue
            if args.mcu_timestamps:
                mcu_ms, vals = vals[0], vals[1:]
                mcu_t0 = mcu_ms if mcu_t0 is None else mcu_t0
                t = (mcu_ms - mcu_t0) / 1000.0
            else:
                t = time.monotonic() - t_start
            ax, ay, az = (v * ka for v in vals[0:3])
            gx, gy, gz = (v * kg for v in vals[3:6])
            row = [t, ax, ay, az, gx, gy, gz]
            if args.with_ref:
                row += [math.radians(v) for v in vals[6:9]]
            out.write(",".join(f"{v:.6f}" for v in row) + "\n")
            count += 1
    return count


def _read_float(path: Path, default: float | None = None) -> float:
    try:
        return float(path.read_text().strip())
    except (OSError, ValueError):
        if default is None:
            raise
        return default


def find_iio_device(root: Path = Path("/sys/bus/iio/devices")) -> Path | None:
    if not root.is_dir():
        return None
    for dev in sorted(root.iterdir()):
        if (dev / "in_accel_x_raw").exists() and (dev / "in_anglvel_x_raw").exists():
            return dev
    return None


def collect_iio(args) -> int:
    dev = find_iio_device() if args.device == "auto" else Path(args.device)
    if dev is None or not (dev / "in_accel_x_raw").exists():
        sys.exit("no IIO accel+gyro device found (is the kernel driver loaded?)")
    a_scale = _read_float(dev / "in_accel_scale", 1.0)
    g_scale = _read_float(dev / "in_anglvel_scale", 1.0)
    period = 1.0 / args.rate
    count = 0
    with open_output(args.out, False) as out:
        t_start = time.monotonic()
        next_tick = t_start
        while True:
            next_tick += period
            now = time.monotonic()
            if now - t_start >= args.seconds:
                break
            raw = [_read_float(dev / f"in_{ch}_{ax}_raw") for ch in ("accel", "anglvel") for ax in "xyz"]
            row = [now - t_start] + [v * a_scale for v in raw[:3]] + [v * g_scale for v in raw[3:]]
            out.write(",".join(f"{v:.6f}" for v in row) + "\n")
            count += 1
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
    print(f"device: {dev}")
    return count


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="backend", required=True)

    s = sub.add_parser("serial", help="read CSV lines from a serial port")
    s.add_argument("--port", required=True)
    s.add_argument("--baud", type=int, default=115200)
    s.add_argument("--accel-unit", choices=ACCEL_UNITS, default="g")
    s.add_argument("--gyro-unit", choices=GYRO_UNITS, default="dps")
    s.add_argument("--mcu-timestamps", action="store_true", help="first field is a ms timestamp")
    s.add_argument("--with-ref", action="store_true", help="last 3 fields are reference roll,pitch,yaw in deg")

    i = sub.add_parser("iio", help="poll a Linux IIO device via sysfs")
    i.add_argument("--device", default="auto")
    i.add_argument("--rate", type=float, default=100.0)

    for p in (s, i):
        p.add_argument("--seconds", type=float, default=30.0)
        p.add_argument("--out", type=Path, default=Path("data/capture.csv"))

    args = ap.parse_args()
    n = collect_serial(args) if args.backend == "serial" else collect_iio(args)
    print(f"captured {n} samples to {args.out}")


if __name__ == "__main__":
    main()
