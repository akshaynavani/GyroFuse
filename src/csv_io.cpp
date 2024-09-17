#include "gyrofuse/csv_io.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <istream>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace gyrofuse {

namespace {

std::string trim(const std::string& s) {
    auto b = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
    auto e = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    return (b < e) ? std::string(b, e) : std::string();
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
}

double toDouble(const std::string& field, std::size_t line) {
    try {
        std::size_t pos = 0;
        double v = std::stod(field, &pos);
        if (pos != field.size()) throw std::invalid_argument("trailing characters");
        return v;
    } catch (const std::exception&) {
        throw std::runtime_error("CSV line " + std::to_string(line) + ": invalid number '" + field + "'");
    }
}

}  // namespace

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> out;
    std::string field;
    std::istringstream ss(line);
    while (std::getline(ss, field, ',')) out.push_back(trim(field));
    if (!line.empty() && line.back() == ',') out.emplace_back();
    return out;
}

CsvReader::CsvReader(std::istream& in) : in_(in) {
    std::string header;
    while (std::getline(in_, header)) {
        ++line_;
        std::string t = trim(header);
        if (t.empty() || t[0] == '#') continue;
        parseHeader(t);
        return;
    }
    throw std::runtime_error("CSV: missing header row");
}

void CsvReader::parseHeader(const std::string& header) {
    auto cols = splitCsvLine(header);
    columns_ = cols.size();
    for (std::size_t i = 0; i < cols.size(); ++i) {
        const std::string name = lower(cols[i]);
        const int idx = static_cast<int>(i);
        if (name == "t" || name == "time" || name == "timestamp") cT_ = idx;
        else if (name == "ax") cAx_ = idx;
        else if (name == "ay") cAy_ = idx;
        else if (name == "az") cAz_ = idx;
        else if (name == "gx") cGx_ = idx;
        else if (name == "gy") cGy_ = idx;
        else if (name == "gz") cGz_ = idx;
        else if (name == "roll_ref") refRoll_ = idx;
        else if (name == "pitch_ref") refPitch_ = idx;
        else if (name == "yaw_ref") refYaw_ = idx;
    }
    for (int c : {cT_, cAx_, cAy_, cAz_, cGx_, cGy_, cGz_}) {
        if (c < 0) throw std::runtime_error("CSV header must contain t,ax,ay,az,gx,gy,gz");
    }
}

std::optional<ImuSample> CsvReader::next() {
    std::string raw;
    while (std::getline(in_, raw)) {
        ++line_;
        std::string t = trim(raw);
        if (t.empty() || t[0] == '#') continue;
        auto f = splitCsvLine(t);
        if (f.size() < columns_) {
            throw std::runtime_error("CSV line " + std::to_string(line_) + ": expected " +
                                     std::to_string(columns_) + " columns");
        }
        auto get = [&](int c) { return toDouble(f[static_cast<std::size_t>(c)], line_); };
        ImuSample s;
        s.t = get(cT_);
        s.accel = {get(cAx_), get(cAy_), get(cAz_)};
        s.gyro = {get(cGx_), get(cGy_), get(cGz_)};
        if (hasReference()) {
            Attitude ref;
            ref.roll = get(refRoll_);
            ref.pitch = get(refPitch_);
            ref.yaw = refYaw_ >= 0 ? get(refYaw_) : 0.0;
            s.reference = ref;
        }
        return s;
    }
    return std::nullopt;
}

std::vector<ImuSample> readCsvFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    CsvReader reader(f);
    std::vector<ImuSample> out;
    while (auto s = reader.next()) out.push_back(*s);
    return out;
}

EstimateWriter::EstimateWriter(std::ostream& out, bool includeReference, bool degrees)
    : out_(out), includeReference_(includeReference), degrees_(degrees) {
    out_ << "t,roll,pitch,yaw,bias_x,bias_y,bias_z";
    if (includeReference_) out_ << ",roll_ref,pitch_ref,yaw_ref";
    out_ << '\n';
}

void EstimateWriter::write(const AttitudeEstimate& est, const std::optional<Attitude>& reference) {
    const double k = degrees_ ? rad2deg(1.0) : 1.0;
    out_ << std::fixed << std::setprecision(6) << est.t << ',' << est.attitude.roll * k << ','
         << est.attitude.pitch * k << ',' << est.attitude.yaw * k << ',' << est.gyroBias.x << ','
         << est.gyroBias.y << ',' << est.gyroBias.z;
    if (includeReference_) {
        Attitude r = reference.value_or(Attitude{});
        out_ << ',' << r.roll * k << ',' << r.pitch * k << ',' << r.yaw * k;
    }
    out_ << '\n';
}

}  // namespace gyrofuse
