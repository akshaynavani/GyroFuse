#include <cmath>

#include "gyrofuse/kalman_filter.hpp"
#include "sim_helpers.hpp"
#include "test_main.hpp"

using namespace gyrofuse;

TEST(kalman_axis_static_measurement_converges) {
    KalmanAxis k;
    k.init(0.0);
    for (int i = 0; i < 2000; ++i) {
        k.predict(0.0, 0.01);
        k.update(0.4);
    }
    CHECK_NEAR(k.angle(), 0.4, 1e-3);
    CHECK_NEAR(k.bias(), 0.0, 1e-3);
}

TEST(kalman_axis_covariance_stays_symmetric_positive) {
    KalmanAxis k;
    gftest::Noise n(3);
    for (int i = 0; i < 5000; ++i) {
        k.predict(n.gaussian(0.1), 0.005);
        k.update(n.gaussian(0.2));
        const auto& P = k.covariance();
        CHECK(P(0, 0) > 0.0);
        CHECK(P(1, 1) > 0.0);
        CHECK_NEAR(P(0, 1), P(1, 0), 1e-12);
        CHECK(P(0, 0) * P(1, 1) - P(0, 1) * P(1, 0) > 0.0);
    }
}

TEST(kalman_axis_wraps_innovation_across_pi) {
    KalmanAxis k;
    k.init(kPi - 0.01);
    k.update(-kPi + 0.01);  // physically 0.02 rad away, not 2*pi
    CHECK(std::fabs(k.lastInnovation()) < 0.05);
}

TEST(kalman_axis_rejects_bad_config) {
    KalmanAxisConfig cfg;
    cfg.rMeasure = 0.0;
    CHECK_THROWS(KalmanAxis(cfg));
}

TEST(kalman_estimates_gyro_bias) {
    gftest::StreamSpec spec;
    spec.rollAmp = 0.0;
    spec.pitchStatic = 0.15;
    spec.duration = 40.0;
    spec.gyroBias = {0.05, -0.03, 0.0};
    spec.gyroNoise = 0.005;
    spec.accelNoise = 0.05;
    KalmanFilter f;
    AttitudeEstimate last;
    for (const auto& s : gftest::rollStream(spec)) last = f.update(s);
    CHECK_NEAR(last.gyroBias.x, 0.05, 5e-3);
    // Pitch bias maps through Euler kinematics at roll = 0, so it is direct.
    CHECK_NEAR(last.gyroBias.y, -0.03, 5e-3);
    CHECK_NEAR(last.attitude.pitch, 0.15, deg2rad(0.5));
}

TEST(kalman_tracks_roll_with_bias_better_than_gyro_only) {
    gftest::StreamSpec spec;
    spec.duration = 30.0;
    spec.gyroBias = {0.03, 0.0, 0.0};
    spec.gyroNoise = 0.01;
    spec.accelNoise = 0.2;
    const auto data = gftest::rollStream(spec);

    KalmanFilter f;
    double sumKf = 0.0, sumGyro = 0.0, gyroOnly = 0.0, prevT = 0.0;
    std::size_t n = 0;
    for (const auto& s : data) {
        const auto e = f.update(s);
        gyroOnly += s.gyro.x * (s.t - prevT);
        prevT = s.t;
        if (s.t < 3.0) continue;
        const double dk = wrapAngle(e.attitude.roll - s.reference->roll);
        const double dg = wrapAngle(gyroOnly - s.reference->roll);
        sumKf += dk * dk;
        sumGyro += dg * dg;
        ++n;
    }
    const double rmseKf = std::sqrt(sumKf / n);
    const double rmseGyro = std::sqrt(sumGyro / n);
    CHECK(rmseKf < deg2rad(1.5));
    CHECK(rmseKf < rmseGyro);
}

TEST(kalman_gates_large_accel_disturbance) {
    KalmanFilter f;
    ImuSample s;
    s.accel = {0, 0, kGravity};
    f.update(s);
    s.t = 0.01;
    s.accel = {3.0 * kGravity, 0, kGravity};
    const auto e = f.update(s);
    CHECK(f.rejectedUpdates() == 1);
    CHECK_NEAR(e.attitude.pitch, 0.0, 1e-9);
}

TEST(kalman_yaw_integrates_gyro) {
    KalmanFilter f;
    ImuSample s;
    s.accel = {0, 0, kGravity};
    s.gyro = {0, 0, 0.2};
    for (int i = 0; i <= 100; ++i) {
        s.t = i * 0.01;
        f.update(s);
    }
    CHECK_NEAR(f.estimate().attitude.yaw, 0.2, 1e-6);
}

TEST(kalman_applies_static_gyro_offset) {
    KalmanFilter f;
    f.setGyroOffset({0, 0, 0.1});
    ImuSample s;
    s.accel = {0, 0, kGravity};
    s.gyro = {0, 0, 0.1};
    for (int i = 0; i <= 100; ++i) {
        s.t = i * 0.01;
        f.update(s);
    }
    CHECK_NEAR(f.estimate().attitude.yaw, 0.0, 1e-9);
    CHECK_NEAR(f.estimate().gyroBias.z, 0.1, 1e-12);
}
