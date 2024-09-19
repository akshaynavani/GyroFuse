#pragma once

#include <chrono>
#include <fstream>
#include <memory>
#include <optional>
#include <string>

#include "gyrofuse/csv_io.hpp"
#include "gyrofuse/imu_sample.hpp"

namespace gyrofuse {

// Pull-based stream of IMU samples.
class ImuSource {
public:
    virtual ~ImuSource() = default;
    // Blocks until the next sample is available; nullopt when exhausted.
    virtual std::optional<ImuSample> read() = 0;
    virtual bool hasReference() const { return false; }
};

// Replays a recorded CSV file.
class CsvSource : public ImuSource {
public:
    explicit CsvSource(const std::string& path);
    std::optional<ImuSample> read() override;
    bool hasReference() const override { return reader_->hasReference(); }

private:
    std::ifstream file_;
    std::unique_ptr<CsvReader> reader_;
};

// Polls a Linux Industrial I/O (IIO) IMU through sysfs, e.g.
// /sys/bus/iio/devices/iio:device0 exposed by the inv_mpu6050, bmi160 or
// st_lsm6dsx drivers. Values are converted with the channel *_scale files,
// which IIO defines as m/s^2 per LSB for accel and rad/s per LSB for gyro.
class IioSource : public ImuSource {
public:
    IioSource(std::string deviceDir, double rateHz, double durationSec = 0.0);
    std::optional<ImuSample> read() override;

    // Returns the first IIO device that exposes both accel and gyro channels.
    static std::optional<std::string> autodetect(const std::string& root = "/sys/bus/iio/devices");

private:
    double readChannel(const std::string& name) const;
    double readScale(const std::string& prefix) const;

    std::string dir_;
    std::chrono::steady_clock::duration period_;
    double duration_;
    double accelScale_ = 1.0;
    double gyroScale_ = 1.0;
    std::chrono::steady_clock::time_point start_;
    std::chrono::steady_clock::time_point nextTick_;
};

}  // namespace gyrofuse
