#include "gyrofuse/kalman_filter.hpp"

#include "gyrofuse/tilt.hpp"

namespace gyrofuse {

KalmanFilter::KalmanFilter(KalmanConfig cfg) : cfg_(cfg), roll_(cfg.axis), pitch_(cfg.axis) {}

void KalmanFilter::reset() {
    roll_ = KalmanAxis(cfg_.axis);
    pitch_ = KalmanAxis(cfg_.axis);
    yaw_ = 0.0;
    t_ = 0.0;
    initialized_ = false;
    rejected_ = 0;
}

AttitudeEstimate KalmanFilter::estimate() const {
    AttitudeEstimate e;
    e.t = t_;
    e.attitude = Attitude{roll_.angle(), pitch_.angle(), yaw_};
    // Report total bias: static calibration offset + online estimate.
    e.gyroBias = gyroOffset_ + Vec3{roll_.bias(), pitch_.bias(), 0.0};
    return e;
}

AttitudeEstimate KalmanFilter::update(const ImuSample& s) {
    const Attitude accelAtt = tiltFromAccel(s.accel);

    if (!initialized_) {
        roll_.init(accelAtt.roll);
        pitch_.init(accelAtt.pitch);
        yaw_ = 0.0;
        t_ = s.t;
        initialized_ = true;
        return estimate();
    }

    const double dt = safeDt(t_, s.t);
    t_ = s.t;
    if (dt == 0.0) return estimate();

    // Euler rates evaluated at the bias-corrected body rates and current attitude.
    const Attitude current{roll_.angle(), pitch_.angle(), yaw_};
    const Vec3 rates = eulerRates(current, s.gyro - gyroOffset_);

    roll_.predict(rates.x, dt);
    pitch_.predict(rates.y, dt);
    yaw_ = wrapAngle(yaw_ + rates.z * dt);

    const double trust = accelTrust(s.accel, cfg_.accelTolerance);
    if (trust < cfg_.minTrust) {
        ++rejected_;
    } else {
        const double rScale = 1.0 / (trust * trust);
        roll_.update(accelAtt.roll, rScale);
        pitch_.update(accelAtt.pitch, rScale);
    }
    return estimate();
}

}  // namespace gyrofuse
