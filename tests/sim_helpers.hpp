#pragma once
// Deterministic synthetic IMU streams for filter tests.

#include <cmath>
#include <cstdint>
#include <vector>

#include "gyrofuse/imu_sample.hpp"

namespace gftest {

using gyrofuse::ImuSample;
using gyrofuse::Vec3;

// Small LCG + Box-Muller so results are identical on every platform.
class Noise {
public:
    explicit Noise(std::uint32_t seed) : state_(seed) {}
    double uniform() {
        state_ = state_ * 1664525u + 1013904223u;
        return (static_cast<double>(state_) + 1.0) / 4294967297.0;
    }
    double gaussian(double sigma) {
        const double u1 = uniform(), u2 = uniform();
        return sigma * std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * gyrofuse::kPi * u2);
    }

private:
    std::uint32_t state_;
};

struct StreamSpec {
    double duration = 20.0;
    double rate = 200.0;
    double rollAmp = 0.5;      // rad
    double rollFreq = 0.25;    // Hz
    double pitchStatic = 0.0;  // rad
    Vec3 gyroBias{};
    double gyroNoise = 0.0;
    double accelNoise = 0.0;
    std::uint32_t seed = 1;
};

// Pure roll oscillation at a constant pitch, with exact body rates.
// With yaw fixed, p = roll_dot and q = r = 0 when pitch is constant and yaw_dot = 0.
inline std::vector<ImuSample> rollStream(const StreamSpec& s) {
    Noise n(s.seed);
    std::vector<ImuSample> out;
    const double dt = 1.0 / s.rate;
    const double w = 2.0 * gyrofuse::kPi * s.rollFreq;
    const std::size_t count = static_cast<std::size_t>(s.duration * s.rate);
    for (std::size_t i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double roll = s.rollAmp * std::sin(w * t);
        const double rollDot = s.rollAmp * w * std::cos(w * t);
        const double p = s.pitchStatic;

        ImuSample smp;
        smp.t = t;
        smp.accel = Vec3{-std::sin(p), std::sin(roll) * std::cos(p), std::cos(roll) * std::cos(p)} *
                        gyrofuse::kGravity +
                    Vec3{n.gaussian(s.accelNoise), n.gaussian(s.accelNoise), n.gaussian(s.accelNoise)};
        smp.gyro = Vec3{rollDot, 0.0, 0.0} + s.gyroBias +
                   Vec3{n.gaussian(s.gyroNoise), n.gaussian(s.gyroNoise), n.gaussian(s.gyroNoise)};
        smp.reference = gyrofuse::Attitude{roll, p, 0.0};
        out.push_back(smp);
    }
    return out;
}

}  // namespace gftest
