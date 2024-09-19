#include "gyrofuse/complementary_filter.hpp"

#include <stdexcept>

#include "gyrofuse/tilt.hpp"

namespace gyrofuse {

ComplementaryFilter::ComplementaryFilter(ComplementaryConfig cfg) : cfg_(cfg) {
    if (cfg_.tau < 0.0) throw std::invalid_argument("ComplementaryFilter: tau must be >= 0");
}

void ComplementaryFilter::reset() {
    est_ = AttitudeEstimate{};
    initialized_ = false;
}

AttitudeEstimate ComplementaryFilter::update(const ImuSample& s) {
    const Attitude accelAtt = tiltFromAccel(s.accel);

    if (!initialized_) {
        est_.t = s.t;
        est_.attitude = accelAtt;
        est_.gyroBias = gyroOffset_;
        initialized_ = true;
        return est_;
    }

    const double dt = safeDt(est_.t, s.t);
    est_.t = s.t;
    if (dt == 0.0) return est_;

    const Vec3 rates = eulerRates(est_.attitude, s.gyro - gyroOffset_);
    Attitude predicted;
    predicted.roll = est_.attitude.roll + rates.x * dt;
    predicted.pitch = est_.attitude.pitch + rates.y * dt;
    predicted.yaw = est_.attitude.yaw + rates.z * dt;

    // Blend weight for the accelerometer, shrunk when |a| deviates from g.
    const double alpha = cfg_.tau / (cfg_.tau + dt);
    const double k = (1.0 - alpha) * accelTrust(s.accel, cfg_.accelTolerance);

    // Blend on the wrapped innovation so the +-pi boundary doesn't cause jumps.
    predicted.roll += k * wrapAngle(accelAtt.roll - predicted.roll);
    predicted.pitch += k * wrapAngle(accelAtt.pitch - predicted.pitch);

    est_.attitude = predicted.wrapped();
    return est_;
}

}  // namespace gyrofuse
