#include "gyrofuse/calibration.hpp"

#include <cmath>

#include "gyrofuse/tilt.hpp"

namespace gyrofuse {

GyroCalibration calibrateGyro(const std::vector<ImuSample>& samples, double window, double maxStd) {
    GyroCalibration cal;
    if (samples.empty()) return cal;

    const double t0 = samples.front().t;
    Vec3 sum, sumSq;
    bool accelOk = true;
    for (const auto& s : samples) {
        if (s.t - t0 >= window) break;
        sum += s.gyro;
        sumSq += Vec3{s.gyro.x * s.gyro.x, s.gyro.y * s.gyro.y, s.gyro.z * s.gyro.z};
        if (accelTrust(s.accel, 0.1) < 0.5) accelOk = false;
        ++cal.samples;
    }
    if (cal.samples == 0) return cal;

    const double n = static_cast<double>(cal.samples);
    cal.bias = sum / n;
    auto sd = [n](double s, double sq) {
        const double var = sq / n - (s / n) * (s / n);
        return var > 0.0 ? std::sqrt(var) : 0.0;
    };
    cal.stddev = {sd(sum.x, sumSq.x), sd(sum.y, sumSq.y), sd(sum.z, sumSq.z)};
    cal.stationary = accelOk && cal.samples >= 10 && cal.stddev.x < maxStd &&
                     cal.stddev.y < maxStd && cal.stddev.z < maxStd;
    return cal;
}

}  // namespace gyrofuse
