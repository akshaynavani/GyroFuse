#pragma once

#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include "gyrofuse/attitude.hpp"
#include "gyrofuse/imu_sample.hpp"

namespace gyrofuse {

// CSV layout (header row required, column order is resolved by name):
//   t,ax,ay,az,gx,gy,gz[,roll_ref,pitch_ref,yaw_ref]
// Lines starting with '#' and blank lines are ignored.
class CsvReader {
public:
    explicit CsvReader(std::istream& in);

    // Returns the next sample, or std::nullopt at end of stream.
    // Throws std::runtime_error on malformed rows.
    std::optional<ImuSample> next();

    bool hasReference() const { return refRoll_ >= 0 && refPitch_ >= 0; }
    std::size_t lineNumber() const { return line_; }

private:
    void parseHeader(const std::string& header);

    std::istream& in_;
    std::size_t line_ = 0;
    int cT_ = -1, cAx_ = -1, cAy_ = -1, cAz_ = -1, cGx_ = -1, cGy_ = -1, cGz_ = -1;
    int refRoll_ = -1, refPitch_ = -1, refYaw_ = -1;
    std::size_t columns_ = 0;
};

std::vector<ImuSample> readCsvFile(const std::string& path);

// Writes filter output: t,roll,pitch,yaw,bias_x,bias_y,bias_z (+ ref columns if present).
class EstimateWriter {
public:
    EstimateWriter(std::ostream& out, bool includeReference, bool degrees);
    void write(const AttitudeEstimate& est, const std::optional<Attitude>& reference);

private:
    std::ostream& out_;
    bool includeReference_;
    bool degrees_;
};

std::vector<std::string> splitCsvLine(const std::string& line);

}  // namespace gyrofuse
