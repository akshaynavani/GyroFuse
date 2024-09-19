#pragma once

#include "gyrofuse/matrix.hpp"

namespace gyrofuse {

struct KalmanAxisConfig {
    double qAngle = 0.001;    // process noise spectral density of the angle, rad^2/s
    double qBias = 0.003;     // random-walk density of the gyro bias, (rad/s)^2/s
    double rMeasure = 0.03;   // accel angle measurement variance, rad^2
    double initialAngleVar = 0.1;
    double initialBiasVar = 0.01;
};

// Single-axis linear Kalman filter with state x = [angle, gyro_bias]^T.
//
//   predict:  angle_k = angle_{k-1} + dt * (rate - bias_{k-1})
//             F = [[1, -dt], [0, 1]],  Q = diag(qAngle, qBias) * dt
//   update:   z = accel angle,  H = [1, 0]
//
// The innovation is wrapped to (-pi, pi] so the filter behaves across the
// +-180 deg boundary.
class KalmanAxis {
public:
    explicit KalmanAxis(KalmanAxisConfig cfg = {});

    void init(double angle, double bias = 0.0);
    void predict(double rate, double dt);
    // `rScale` multiplies the measurement variance (> 1 means trust it less).
    void update(double measuredAngle, double rScale = 1.0);

    double angle() const { return x_(0, 0); }
    double bias() const { return x_(1, 0); }
    const Mat2& covariance() const { return P_; }
    double lastInnovation() const { return innovation_; }
    const KalmanAxisConfig& config() const { return cfg_; }

private:
    KalmanAxisConfig cfg_;
    Vec2 x_;
    Mat2 P_;
    double innovation_ = 0.0;
};

}  // namespace gyrofuse
