#pragma once

#include <memory>
#include <string>

#include "gyrofuse/attitude.hpp"
#include "gyrofuse/imu_sample.hpp"

namespace gyrofuse {

// Common interface for attitude estimators.
class AttitudeFilter {
public:
    virtual ~AttitudeFilter() = default;

    // Feed one sample; returns the updated estimate. The first call seeds
    // the state from the accelerometer tilt.
    virtual AttitudeEstimate update(const ImuSample& sample) = 0;

    virtual AttitudeEstimate estimate() const = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;

    // Gyro bias subtracted before integration (e.g. from a static calibration).
    void setGyroOffset(const Vec3& offset) { gyroOffset_ = offset; }
    const Vec3& gyroOffset() const { return gyroOffset_; }

protected:
    // Sample period with protection against duplicate/out-of-order timestamps
    // and long gaps (which would otherwise inject a huge integration step).
    static double safeDt(double prevT, double t, double maxDt = 0.1) {
        double dt = t - prevT;
        if (!(dt > 0.0)) return 0.0;
        return dt > maxDt ? maxDt : dt;
    }

    Vec3 gyroOffset_;
};

}  // namespace gyrofuse
