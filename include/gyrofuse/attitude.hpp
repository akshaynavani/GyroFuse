#pragma once

#include "gyrofuse/vec3.hpp"

namespace gyrofuse {

// Euler attitude in radians, aerospace ZYX convention (yaw -> pitch -> roll).
struct Attitude {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    Attitude wrapped() const { return {wrapAngle(roll), wrapAngle(pitch), wrapAngle(yaw)}; }
};

// Filter output: attitude plus the current gyro bias estimate (rad/s).
struct AttitudeEstimate {
    double t = 0.0;
    Attitude attitude;
    Vec3 gyroBias;
};

}  // namespace gyrofuse
