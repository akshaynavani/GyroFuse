#pragma once

#include "gyrofuse/filter.hpp"
#include "gyrofuse/kalman_axis.hpp"

namespace gyrofuse {

struct KalmanConfig {
    KalmanAxisConfig axis;
    // Accel magnitude tolerance (fraction of g). Within tolerance the
    // measurement variance is inflated by 1/trust^2; below `minTrust` the
    // correction is skipped entirely.
    double accelTolerance = 0.2;
    double minTrust = 0.1;
};

// Attitude estimator built from independent angle/bias Kalman filters on roll
// and pitch. Body rates are mapped to Euler rates before prediction, so the
// estimated biases live in Euler-rate space (equal to body-frame gyro bias
// near level attitude). Yaw is gyro integration only because gravity carries
// no heading information.
class KalmanFilter : public AttitudeFilter {
public:
    explicit KalmanFilter(KalmanConfig cfg = {});

    AttitudeEstimate update(const ImuSample& sample) override;
    AttitudeEstimate estimate() const override;
    void reset() override;
    std::string name() const override { return "kalman"; }

    const KalmanAxis& rollAxis() const { return roll_; }
    const KalmanAxis& pitchAxis() const { return pitch_; }
    const KalmanConfig& config() const { return cfg_; }
    // Number of accelerometer corrections rejected by magnitude gating.
    std::size_t rejectedUpdates() const { return rejected_; }

private:
    KalmanConfig cfg_;
    KalmanAxis roll_;
    KalmanAxis pitch_;
    double yaw_ = 0.0;
    double t_ = 0.0;
    bool initialized_ = false;
    std::size_t rejected_ = 0;
};

}  // namespace gyrofuse
