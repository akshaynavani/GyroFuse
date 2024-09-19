#include <stdexcept>

#include "gyrofuse/imu_source.hpp"

namespace gyrofuse {

CsvSource::CsvSource(const std::string& path) : file_(path) {
    if (!file_) throw std::runtime_error("cannot open " + path);
    reader_ = std::make_unique<CsvReader>(file_);
}

std::optional<ImuSample> CsvSource::read() { return reader_->next(); }

}  // namespace gyrofuse
