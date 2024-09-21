#include <cmath>

#include "gyrofuse/complementary_filter.hpp"
#include "sim_helpers.hpp"
#include "test_main.hpp"

using namespace gyrofuse;

namespace {
double rollRmse(AttitudeFilter& f, const std::vector<ImuSample>& data, double skipSec) {
    double sum = 0.0;
    std::size_t n = 0;
    for (const auto& s : data) {
        const auto e = f.update(s);
        if (s.t < skipSec) continue;
        const double d = wrapAngle(e.attitude.roll - s.reference->roll);
        sum += d * d;
        ++n;
    }
    return std::sqrt(sum / static_cast<double>(n));
}
}  // namespace

TEST(complementary_initializes_from_accel) {
    ComplementaryFilter f;
    ImuSample s;
    s.accel = Vec3{0, std::sin(0.3), std::cos(0.3)} * kGravity;
    const auto e = f.update(s);
    CHECK_NEAR(e.attitude.roll, 0.3, 1e-9);
    CHECK_NEAR(e.attitude.pitch, 0.0, 1e-9);
}

TEST(complementary_tracks_noisy_roll) {
    gftest::StreamSpec spec;
    spec.gyroNoise = 0.01;
    spec.accelNoise = 0.2;
    spec.pitchStatic = 0.2;
    ComplementaryFilter f;
    const double rmse = rollRmse(f, gftest::rollStream(spec), 2.0);
    CHECK(rmse < deg2rad(2.0));
}

TEST(complementary_bounds_bias_drift) {
    // Static sensor with a gyro bias: the error settles near bias * tau
    // instead of growing without bound like pure integration.
    gftest::StreamSpec spec;
    spec.rollAmp = 0.0;
    spec.duration = 30.0;
    spec.gyroBias = {0.02, 0.0, 0.0};
    ComplementaryConfig cfg;
    cfg.tau = 0.5;
    ComplementaryFilter f(cfg);
    AttitudeEstimate last;
    for (const auto& s : gftest::rollStream(spec)) last = f.update(s);
    CHECK_NEAR(last.attitude.roll, 0.02 * cfg.tau, 2e-3);
}

TEST(complementary_rejects_linear_acceleration_spike) {
    ComplementaryFilter f;
    ImuSample s;
    s.accel = {0, 0, kGravity};
    f.update(s);
    s.t = 0.01;
    f.update(s);
    // 2 g sideways shove: accel tilt would read ~63 deg roll.
    s.t = 0.02;
    s.accel = {0, 2.0 * kGravity, kGravity};
    const auto e = f.update(s);
    CHECK_NEAR(e.attitude.roll, 0.0, 1e-9);
}

TEST(complementary_ignores_duplicate_timestamps) {
    ComplementaryFilter f;
    ImuSample s;
    s.accel = {0, 0, kGravity};
    f.update(s);
    s.gyro = {5.0, 0, 0};
    const auto e = f.update(s);  // dt == 0
    CHECK_NEAR(e.attitude.roll, 0.0, 1e-12);
}

TEST(complementary_reset_clears_state) {
    ComplementaryFilter f;
    ImuSample s;
    s.accel = Vec3{0, std::sin(0.5), std::cos(0.5)} * kGravity;
    f.update(s);
    f.reset();
    s.accel = {0, 0, kGravity};
    CHECK_NEAR(f.update(s).attitude.roll, 0.0, 1e-12);
}
