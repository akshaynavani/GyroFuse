#pragma once

#include "gyrofuse/filter.hpp"

namespace gyrofuse {

struct ComplementaryConfig {
    // Crossover time constant in seconds. alpha = tau / (tau + dt), so the
    // gyro dominates for motion faster than ~1/tau and the accel corrects drift.
    double tau = 0.5;
    // Accelerometer magnitude tolerance (fraction of g) beyond which the accel
    // correction is progressively disabled. <= 0 disables gating.
    double accelTolerance = 0.15;
};

// Classic complementary filter:
//   angle = alpha * (angle + rate * dt) + (1 - alpha) * accel_angle
// Roll/pitch are corrected by gravity; yaw is integrated gyro only.
class ComplementaryFilter : public AttitudeFilter {
public:
    explicit ComplementaryFilter(ComplementaryConfig cfg = {});

    AttitudeEstimate update(const ImuSample& sample) override;
    AttitudeEstimate estimate() const override { return est_; }
    void reset() override;
    std::string name() const override { return "complementary"; }

    const ComplementaryConfig& config() const { return cfg_; }

private:
    ComplementaryConfig cfg_;
    AttitudeEstimate est_;
    bool initialized_ = false;
};

}  // namespace gyrofuse
