#pragma once

#include <vector>

#include "gyrofuse/imu_sample.hpp"

namespace gyrofuse {

struct GyroCalibration {
    Vec3 bias;            // mean gyro reading while still, rad/s
    Vec3 stddev;          // per-axis noise, rad/s
    std::size_t samples = 0;
    bool stationary = false;  // false if motion was detected during the window
};

// Estimates the static gyro bias from the samples in [t0, t0 + window).
// The device is considered stationary when every axis' standard deviation
// is below `maxStd` rad/s and the accel magnitude stays near 1 g.
GyroCalibration calibrateGyro(const std::vector<ImuSample>& samples, double window = 2.0,
                              double maxStd = 0.02);

}  // namespace gyrofuse
