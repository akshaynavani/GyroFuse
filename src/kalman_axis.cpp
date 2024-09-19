#include "gyrofuse/kalman_axis.hpp"

#include <stdexcept>

#include "gyrofuse/vec3.hpp"

namespace gyrofuse {

KalmanAxis::KalmanAxis(KalmanAxisConfig cfg) : cfg_(cfg) {
    if (cfg_.qAngle < 0.0 || cfg_.qBias < 0.0 || cfg_.rMeasure <= 0.0) {
        throw std::invalid_argument("KalmanAxis: noise parameters must be positive");
    }
    init(0.0);
}

void KalmanAxis::init(double angle, double bias) {
    x_ = Vec2{angle, bias};
    P_ = Mat2{cfg_.initialAngleVar, 0.0, 0.0, cfg_.initialBiasVar};
    innovation_ = 0.0;
}

void KalmanAxis::predict(double rate, double dt) {
    if (dt <= 0.0) return;
    const Mat2 F{1.0, -dt, 0.0, 1.0};
    const Mat2 Q{cfg_.qAngle * dt, 0.0, 0.0, cfg_.qBias * dt};

    x_(0, 0) = wrapAngle(x_(0, 0) + dt * (rate - x_(1, 0)));
    P_ = F * P_ * F.transpose() + Q;
}

void KalmanAxis::update(double measuredAngle, double rScale) {
    if (rScale <= 0.0) rScale = 1.0;
    const double R = cfg_.rMeasure * rScale;

    // H = [1 0] so S and K reduce to scalar/column forms.
    innovation_ = wrapAngle(measuredAngle - x_(0, 0));
    const double S = P_(0, 0) + R;
    const double k0 = P_(0, 0) / S;
    const double k1 = P_(1, 0) / S;

    x_(0, 0) = wrapAngle(x_(0, 0) + k0 * innovation_);
    x_(1, 0) += k1 * innovation_;

    // Joseph form keeps P symmetric positive semi-definite under rounding:
    //   P = (I - K H) P (I - K H)^T + K R K^T
    const Mat2 IKH{1.0 - k0, 0.0, -k1, 1.0};
    const Mat2 KRK{k0 * k0 * R, k0 * k1 * R, k1 * k0 * R, k1 * k1 * R};
    P_ = IKH * P_ * IKH.transpose() + KRK;
}

}  // namespace gyrofuse
