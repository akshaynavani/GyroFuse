#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <thread>

#include "gyrofuse/imu_source.hpp"

namespace fs = std::filesystem;

namespace gyrofuse {

namespace {

std::optional<double> readNumber(const fs::path& p) {
    std::ifstream f(p);
    double v = 0.0;
    if (f >> v) return v;
    return std::nullopt;
}

}  // namespace

IioSource::IioSource(std::string deviceDir, double rateHz, double durationSec)
    : dir_(std::move(deviceDir)), duration_(durationSec) {
    if (rateHz <= 0.0) throw std::invalid_argument("IioSource: rate must be positive");
    if (!fs::exists(fs::path(dir_) / "in_accel_x_raw") || !fs::exists(fs::path(dir_) / "in_anglvel_x_raw")) {
        throw std::runtime_error("IioSource: " + dir_ + " does not expose accel + anglvel channels");
    }
    period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / rateHz));
    accelScale_ = readScale("in_accel");
    gyroScale_ = readScale("in_anglvel");
    start_ = std::chrono::steady_clock::now();
    nextTick_ = start_;
}

double IioSource::readScale(const std::string& prefix) const {
    // Shared scale (in_accel_scale) is most common; fall back to per-axis.
    for (const char* name : {"_scale", "_x_scale"}) {
        if (auto v = readNumber(fs::path(dir_) / (prefix + name))) return *v;
    }
    return 1.0;
}

double IioSource::readChannel(const std::string& name) const {
    auto v = readNumber(fs::path(dir_) / name);
    if (!v) throw std::runtime_error("IioSource: failed to read " + name);
    return *v;
}

std::optional<ImuSample> IioSource::read() {
    std::this_thread::sleep_until(nextTick_);
    nextTick_ += period_;

    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - start_).count();
    if (duration_ > 0.0 && t > duration_) return std::nullopt;

    ImuSample s;
    s.t = t;
    s.accel = Vec3{readChannel("in_accel_x_raw"), readChannel("in_accel_y_raw"),
                   readChannel("in_accel_z_raw")} * accelScale_;
    s.gyro = Vec3{readChannel("in_anglvel_x_raw"), readChannel("in_anglvel_y_raw"),
                  readChannel("in_anglvel_z_raw")} * gyroScale_;
    return s;
}

std::optional<std::string> IioSource::autodetect(const std::string& root) {
    std::error_code ec;
    if (!fs::is_directory(root, ec)) return std::nullopt;
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        const auto& p = entry.path();
        if (fs::exists(p / "in_accel_x_raw") && fs::exists(p / "in_anglvel_x_raw")) return p.string();
    }
    return std::nullopt;
}

}  // namespace gyrofuse
