// gyrofuse — run an attitude filter over recorded or live IMU data.
//
//   gyrofuse --input data.csv --filter kalman --output est.csv
//   gyrofuse --iio auto --rate 200 --duration 30 --calibrate 2 --degrees

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "gyrofuse/calibration.hpp"
#include "gyrofuse/complementary_filter.hpp"
#include "gyrofuse/csv_io.hpp"
#include "gyrofuse/imu_source.hpp"
#include "gyrofuse/kalman_filter.hpp"

using namespace gyrofuse;

namespace {

struct Options {
    std::string input;
    std::string iio;
    std::string output;
    std::string filter = "kalman";
    double rate = 100.0;
    double duration = 0.0;
    double calibrate = 0.0;
    bool degrees = false;
    ComplementaryConfig comp;
    KalmanConfig kalman;
};

void usage(const char* prog) {
    std::cerr
        << "Usage: " << prog << " (--input FILE.csv | --iio DIR|auto) [options]\n\n"
        << "Sources:\n"
        << "  --input FILE        replay CSV (t,ax,ay,az,gx,gy,gz[,roll_ref,pitch_ref,yaw_ref])\n"
        << "  --iio DIR|auto      poll a Linux IIO IMU via sysfs\n"
        << "  --rate HZ           IIO polling rate (default 100)\n"
        << "  --duration S        stop IIO capture after S seconds (default: run forever)\n\n"
        << "Filter:\n"
        << "  --filter NAME       kalman | complementary (default kalman)\n"
        << "  --tau S             complementary time constant (default 0.5)\n"
        << "  --q-angle V         Kalman angle process noise (default 0.001)\n"
        << "  --q-bias V          Kalman bias random walk (default 0.003)\n"
        << "  --r-measure V       Kalman accel measurement variance (default 0.03)\n"
        << "  --accel-tol F       accel magnitude gating tolerance, fraction of g\n"
        << "  --calibrate S       estimate gyro bias from the first S seconds (device still)\n\n"
        << "Output:\n"
        << "  --output FILE       write estimates CSV (default stdout)\n"
        << "  --degrees           write angles in degrees instead of radians\n";
}

Options parseArgs(int argc, char** argv) {
    Options o;
    auto need = [&](int& i) -> std::string {
        if (i + 1 >= argc) {
            std::cerr << "missing value for " << argv[i] << "\n";
            std::exit(2);
        }
        return argv[++i];
    };
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") { usage(argv[0]); std::exit(0); }
        else if (a == "--input") o.input = need(i);
        else if (a == "--iio") o.iio = need(i);
        else if (a == "--output") o.output = need(i);
        else if (a == "--filter") o.filter = need(i);
        else if (a == "--rate") o.rate = std::stod(need(i));
        else if (a == "--duration") o.duration = std::stod(need(i));
        else if (a == "--calibrate") o.calibrate = std::stod(need(i));
        else if (a == "--degrees") o.degrees = true;
        else if (a == "--tau") o.comp.tau = std::stod(need(i));
        else if (a == "--q-angle") o.kalman.axis.qAngle = std::stod(need(i));
        else if (a == "--q-bias") o.kalman.axis.qBias = std::stod(need(i));
        else if (a == "--r-measure") o.kalman.axis.rMeasure = std::stod(need(i));
        else if (a == "--accel-tol") {
            const double tol = std::stod(need(i));
            o.comp.accelTolerance = tol;
            o.kalman.accelTolerance = tol;
        } else {
            std::cerr << "unknown option: " << a << "\n";
            usage(argv[0]);
            std::exit(2);
        }
    }
    if (o.input.empty() == o.iio.empty()) {
        std::cerr << "exactly one of --input or --iio is required\n";
        usage(argv[0]);
        std::exit(2);
    }
    return o;
}

std::unique_ptr<AttitudeFilter> makeFilter(const Options& o) {
    if (o.filter == "kalman") return std::make_unique<KalmanFilter>(o.kalman);
    if (o.filter == "complementary") return std::make_unique<ComplementaryFilter>(o.comp);
    throw std::invalid_argument("unknown filter '" + o.filter + "'");
}

std::unique_ptr<ImuSource> makeSource(const Options& o) {
    if (!o.input.empty()) return std::make_unique<CsvSource>(o.input);
    std::string dir = o.iio;
    if (dir == "auto") {
        auto found = IioSource::autodetect();
        if (!found) throw std::runtime_error("no IIO device with accel+gyro channels found");
        dir = *found;
        std::cerr << "using IIO device " << dir << "\n";
    }
    return std::make_unique<IioSource>(dir, o.rate, o.duration);
}

struct ErrorStats {
    double sumRoll = 0.0, sumPitch = 0.0;
    std::size_t n = 0;
    void add(const Attitude& est, const Attitude& ref) {
        const double dr = wrapAngle(est.roll - ref.roll);
        const double dp = wrapAngle(est.pitch - ref.pitch);
        sumRoll += dr * dr;
        sumPitch += dp * dp;
        ++n;
    }
};

}  // namespace

int main(int argc, char** argv) {
    const Options opts = parseArgs(argc, argv);
    try {
        auto source = makeSource(opts);
        auto filter = makeFilter(opts);

        std::ofstream file;
        if (!opts.output.empty()) {
            file.open(opts.output);
            if (!file) throw std::runtime_error("cannot write " + opts.output);
        }
        std::ostream& out = opts.output.empty() ? std::cout : file;
        EstimateWriter writer(out, source->hasReference(), opts.degrees);

        // Optional calibration: buffer the first window, estimate bias, then
        // replay the buffered samples through the filter.
        std::vector<ImuSample> pending;
        if (opts.calibrate > 0.0) {
            while (auto s = source->read()) {
                pending.push_back(*s);
                if (s->t - pending.front().t >= opts.calibrate) break;
            }
            const auto cal = calibrateGyro(pending, opts.calibrate);
            filter->setGyroOffset(cal.bias);
            std::cerr << "gyro bias [rad/s]: " << cal.bias.x << ", " << cal.bias.y << ", "
                      << cal.bias.z << " from " << cal.samples << " samples"
                      << (cal.stationary ? "" : " (WARNING: motion detected during calibration)")
                      << "\n";
        }

        ErrorStats stats;
        std::size_t count = 0;
        auto process = [&](const ImuSample& s) {
            const auto est = filter->update(s);
            writer.write(est, s.reference);
            if (s.reference) stats.add(est.attitude, *s.reference);
            ++count;
        };
        for (const auto& s : pending) process(s);
        while (auto s = source->read()) process(*s);
        out.flush();

        std::cerr << filter->name() << ": processed " << count << " samples\n";
        if (stats.n > 0) {
            const double k = rad2deg(1.0);
            std::cerr << "RMSE roll  = " << std::sqrt(stats.sumRoll / stats.n) * k << " deg\n"
                      << "RMSE pitch = " << std::sqrt(stats.sumPitch / stats.n) * k << " deg\n";
        }
        if (auto* kf = dynamic_cast<KalmanFilter*>(filter.get())) {
            std::cerr << "accel updates rejected by gating: " << kf->rejectedUpdates() << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
