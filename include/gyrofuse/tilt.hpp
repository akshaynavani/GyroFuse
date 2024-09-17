#pragma once

#include "gyrofuse/attitude.hpp"
#include "gyrofuse/vec3.hpp"

namespace gyrofuse {

// Roll/pitch implied by the gravity vector seen by the accelerometer.
// Valid only when the body is not undergoing significant linear acceleration.
// Yaw is unobservable from gravity and returned as 0.
Attitude tiltFromAccel(const Vec3& accel);

// Converts body angular rates (p, q, r) to Euler angle rates (roll_dot,
// pitch_dot, yaw_dot) for the ZYX convention. Pitch is clamped away from
// +-90 deg to avoid the gimbal-lock singularity.
Vec3 eulerRates(const Attitude& att, const Vec3& gyro);

// Confidence weight in [0, 1] for an accelerometer reading: 1 when |a| == g,
// falling linearly to 0 when the deviation reaches `tolerance` (fraction of g).
double accelTrust(const Vec3& accel, double tolerance);

}  // namespace gyrofuse
