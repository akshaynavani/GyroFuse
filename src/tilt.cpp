#include "gyrofuse/tilt.hpp"

#include <algorithm>
#include <cmath>

namespace gyrofuse {

Attitude tiltFromAccel(const Vec3& a) {
    Attitude att;
    att.roll = std::atan2(a.y, a.z);
    att.pitch = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
    att.yaw = 0.0;
    return att;
}

Vec3 eulerRates(const Attitude& att, const Vec3& w) {
    constexpr double kMaxPitch = deg2rad(89.5);
    const double pitch = std::clamp(att.pitch, -kMaxPitch, kMaxPitch);
    const double sr = std::sin(att.roll), cr = std::cos(att.roll);
    const double cp = std::cos(pitch), tp = std::tan(pitch);

    Vec3 rates;
    rates.x = w.x + sr * tp * w.y + cr * tp * w.z;
    rates.y = cr * w.y - sr * w.z;
    rates.z = (sr * w.y + cr * w.z) / cp;
    return rates;
}

double accelTrust(const Vec3& accel, double tolerance) {
    if (tolerance <= 0.0) return 1.0;
    const double dev = std::fabs(accel.norm() - kGravity) / kGravity;
    return std::clamp(1.0 - dev / tolerance, 0.0, 1.0);
}

}  // namespace gyrofuse
