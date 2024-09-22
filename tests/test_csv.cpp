#include <sstream>

#include "gyrofuse/calibration.hpp"
#include "gyrofuse/csv_io.hpp"
#include "test_main.hpp"

using namespace gyrofuse;

TEST(csv_reads_samples_and_reference) {
    std::istringstream in(
        "# comment line\n"
        "t,ax,ay,az,gx,gy,gz,roll_ref,pitch_ref,yaw_ref\n"
        "0.00,0,0,9.8,0.1,0.2,0.3,0.01,0.02,0.03\n"
        "\n"
        "0.01, 1, 2, 3, 4, 5, 6, 0, 0, 0\n");
    CsvReader r(in);
    CHECK(r.hasReference());
    auto a = r.next();
    CHECK(a.has_value());
    CHECK_NEAR(a->accel.z, 9.8, 1e-12);
    CHECK_NEAR(a->gyro.z, 0.3, 1e-12);
    CHECK(a->reference.has_value());
    CHECK_NEAR(a->reference->pitch, 0.02, 1e-12);
    auto b = r.next();
    CHECK(b.has_value());
    CHECK_NEAR(b->t, 0.01, 1e-12);
    CHECK_NEAR(b->gyro.x, 4.0, 1e-12);
    CHECK(!r.next().has_value());
}

TEST(csv_column_order_is_resolved_by_name) {
    std::istringstream in("gx,gy,gz,ax,ay,az,t\n1,2,3,4,5,6,7\n");
    CsvReader r(in);
    CHECK(!r.hasReference());
    auto s = r.next();
    CHECK_NEAR(s->t, 7.0, 1e-12);
    CHECK_NEAR(s->accel.x, 4.0, 1e-12);
    CHECK_NEAR(s->gyro.x, 1.0, 1e-12);
    CHECK(!s->reference.has_value());
}

TEST(csv_rejects_bad_input) {
    std::istringstream missing("t,ax,ay\n");
    CHECK_THROWS(CsvReader{missing});

    std::istringstream bad("t,ax,ay,az,gx,gy,gz\n0,1,2,x,4,5,6\n");
    CsvReader r(bad);
    CHECK_THROWS(r.next());

    std::istringstream shortRow("t,ax,ay,az,gx,gy,gz\n0,1,2\n");
    CsvReader r2(shortRow);
    CHECK_THROWS(r2.next());
}

TEST(estimate_writer_formats_degrees) {
    std::ostringstream out;
    EstimateWriter w(out, true, true);
    AttitudeEstimate e;
    e.t = 1.0;
    e.attitude.roll = kPi / 2;
    w.write(e, Attitude{kPi, 0, 0});
    const std::string s = out.str();
    CHECK(s.find("t,roll,pitch,yaw,bias_x,bias_y,bias_z,roll_ref,pitch_ref,yaw_ref") == 0);
    CHECK(s.find("1.000000,90.000000") != std::string::npos);
    CHECK(s.find(",180.000000,") != std::string::npos);
}

TEST(calibration_estimates_static_bias) {
    std::vector<ImuSample> data;
    for (int i = 0; i < 400; ++i) {
        ImuSample s;
        s.t = i * 0.005;
        s.accel = {0, 0, kGravity};
        s.gyro = {0.01 + ((i % 2) ? 1e-3 : -1e-3), -0.02, 0.005};
        data.push_back(s);
    }
    const auto cal = calibrateGyro(data, 0.9975);
    CHECK(cal.samples == 200);
    CHECK(cal.stationary);
    CHECK_NEAR(cal.bias.x, 0.01, 1e-9);
    CHECK_NEAR(cal.bias.y, -0.02, 1e-9);
    CHECK_NEAR(cal.stddev.x, 1e-3, 1e-6);

    data[10].gyro.x = 2.0;  // motion during the window
    CHECK(!calibrateGyro(data, 0.9975).stationary);
}
