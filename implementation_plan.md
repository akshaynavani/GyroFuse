# GyroFuse — Implementation Plan

Kalman-filtered IMU motion tracking: a C++17 sensor-fusion library and CLI that fuses
3-axis accelerometer + gyroscope data into roll/pitch(/yaw) attitude estimates using a
complementary filter and a Kalman filter, plus Python tooling for data collection,
visualization, and validation against ground truth.

## Design decisions

- **Language/build:** C++17, CMake (>= 3.16), no mandatory third-party dependencies.
  Small in-house fixed-size matrix template keeps the Kalman math readable and header-only.
- **Platform:** Linux first. Live sensor input via the Linux IIO sysfs interface
  (`/sys/bus/iio/devices/iio:deviceN`), which covers MPU6050/MPU9250/BMI160/LSM6DS* drivers.
  Offline input via CSV so everything works on any machine.
- **Filters:**
  - Complementary filter: high-pass gyro integration + low-pass accelerometer tilt.
  - Kalman filter: per-axis 2-state `[angle, gyro_bias]` linear KF (the classic, robust choice
    for embedded attitude), with tunable process/measurement noise and accel-magnitude gating
    to reject measurements during linear acceleration.
  - Yaw is gyro-integrated only (no magnetometer) — documented as a known limitation.
- **Data format (CSV):** `t,ax,ay,az,gx,gy,gz[,roll_ref,pitch_ref,yaw_ref]` with
  `t` in seconds, accel in m/s^2, gyro in rad/s, reference angles in rad.
- **Python:** numpy/pandas/matplotlib scripts for synthetic data with ground truth,
  live/serial capture, plotting, and RMSE validation.
- **Testing:** dependency-free C++ unit tests registered with CTest; pytest for Python tools.

## Phase 1 — Core foundations
Deliverables: CMake project, math types, IMU sample model, CSV I/O.
- `CMakeLists.txt`
- `include/gyrofuse/vec3.hpp`, `include/gyrofuse/matrix.hpp`
- `include/gyrofuse/imu_sample.hpp`, `include/gyrofuse/attitude.hpp`
- `include/gyrofuse/csv_io.hpp`, `src/csv_io.cpp`

## Phase 2 — Complementary filter
Deliverables: accelerometer tilt computation, common filter interface, complementary filter.
- `include/gyrofuse/tilt.hpp`, `src/tilt.cpp`
- `include/gyrofuse/filter.hpp`
- `include/gyrofuse/complementary_filter.hpp`, `src/complementary_filter.cpp`

## Phase 3 — Kalman filter, sensor sources and CLI
Deliverables: 2-state per-axis KF with bias estimation and accel gating, Kalman attitude
filter, gyro calibration, IIO + CSV sources, `gyrofuse` CLI.
- `include/gyrofuse/kalman_axis.hpp`, `src/kalman_axis.cpp`
- `include/gyrofuse/kalman_filter.hpp`, `src/kalman_filter.cpp`
- `include/gyrofuse/calibration.hpp`, `src/calibration.cpp`
- `include/gyrofuse/imu_source.hpp`, `src/csv_source.cpp`, `src/iio_source.cpp`
- `apps/gyrofuse_cli.cpp`

## Phase 4 — Python tooling
Deliverables: synthetic dataset generator, live capture, plotting, validation report.
- `tools/requirements.txt`
- `tools/gyrofuse_tools/{__init__,io,metrics}.py`
- `tools/simulate.py`, `tools/collect.py`, `tools/visualize.py`, `tools/validate.py`

## Phase 5 — Tests
Deliverables: C++ unit tests via CTest, Python tests via pytest.
- `tests/test_main.hpp`, `tests/test_matrix.cpp`, `tests/test_tilt.cpp`,
  `tests/test_complementary.cpp`, `tests/test_kalman.cpp`, `tests/test_csv.cpp`
- `tools/tests/test_metrics.py`, `tools/tests/test_simulate.py`

Wrap-up: `.gitignore`, `README.md` (architecture diagram, run + test instructions).
