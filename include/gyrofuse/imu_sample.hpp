#pragma once

#include <optional>

#include "gyrofuse/attitude.hpp"
#include "gyrofuse/vec3.hpp"

namespace gyrofuse {

// One time-stamped IMU reading.
//   t     : seconds (monotonic)
//   accel : specific force in the body frame, m/s^2 (reads +g on z when level)
//   gyro  : angular rate in the body frame, rad/s
//   reference : optional ground-truth attitude for validation datasets
struct ImuSample {
    double t = 0.0;
    Vec3 accel;
    Vec3 gyro;
    std::optional<Attitude> reference;
};

}  // namespace gyrofuse
