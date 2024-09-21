#include <cmath>

#include "gyrofuse/tilt.hpp"
#include "test_main.hpp"

using namespace gyrofuse;

namespace {
// Specific force seen by a still sensor at the given roll/pitch (ZYX).
Vec3 gravityBody(double roll, double pitch) {
    return Vec3{-std::sin(pitch), std::sin(roll) * std::cos(pitch), std::cos(roll) * std::cos(pitch)} * kGravity;
}
}  // namespace

TEST(tilt_level) {
    const Attitude a = tiltFromAccel({0, 0, kGravity});
    CHECK_NEAR(a.roll, 0.0, 1e-12);
    CHECK_NEAR(a.pitch, 0.0, 1e-12);
}

TEST(tilt_recovers_roll_and_pitch) {
    for (double r : {-1.2, -0.4, 0.0, 0.3, 1.0}) {
        for (double p : {-1.0, -0.2, 0.0, 0.5, 1.3}) {
            const Attitude a = tiltFromAccel(gravityBody(r, p));
            CHECK_NEAR(a.roll, r, 1e-9);
            CHECK_NEAR(a.pitch, p, 1e-9);
        }
    }
}

TEST(euler_rates_identity_when_level) {
    const Vec3 rates = eulerRates(Attitude{}, {0.1, -0.2, 0.3});
    CHECK_NEAR(rates.x, 0.1, 1e-12);
    CHECK_NEAR(rates.y, -0.2, 1e-12);
    CHECK_NEAR(rates.z, 0.3, 1e-12);
}

TEST(euler_rates_rolled_90_maps_q_to_yaw) {
    // Rolled 90 deg: body y axis points along world -z... so body q drives yaw.
    const Vec3 rates = eulerRates(Attitude{kPi / 2, 0, 0}, {0, 1.0, 0});
    CHECK_NEAR(rates.y, 0.0, 1e-9);
    CHECK_NEAR(rates.z, 1.0, 1e-9);
}

TEST(accel_trust_gating) {
    CHECK_NEAR(accelTrust({0, 0, kGravity}, 0.2), 1.0, 1e-12);
    CHECK_NEAR(accelTrust({0, 0, kGravity * 1.1}, 0.2), 0.5, 1e-9);
    CHECK_NEAR(accelTrust({0, 0, kGravity * 1.5}, 0.2), 0.0, 1e-12);
    CHECK_NEAR(accelTrust({0, 0, 0}, 0.0), 1.0, 1e-12);
}
