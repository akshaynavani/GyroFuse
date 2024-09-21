#include "gyrofuse/matrix.hpp"
#include "gyrofuse/vec3.hpp"
#include "test_main.hpp"

using namespace gyrofuse;

TEST(matrix_multiply_and_transpose) {
    const Matrix<2, 3> a{1, 2, 3, 4, 5, 6};
    const Matrix<3, 2> b = a.transpose();
    const Mat2 c = a * b;  // [[14, 32], [32, 77]]
    CHECK_NEAR(c(0, 0), 14.0, 1e-12);
    CHECK_NEAR(c(0, 1), 32.0, 1e-12);
    CHECK_NEAR(c(1, 0), 32.0, 1e-12);
    CHECK_NEAR(c(1, 1), 77.0, 1e-12);
}

TEST(matrix_inverse_2x2) {
    const Mat2 m{4, 7, 2, 6};
    const Mat2 i = m * inverse(m);
    CHECK_NEAR(i(0, 0), 1.0, 1e-12);
    CHECK_NEAR(i(0, 1), 0.0, 1e-12);
    CHECK_NEAR(i(1, 0), 0.0, 1e-12);
    CHECK_NEAR(i(1, 1), 1.0, 1e-12);
    CHECK_THROWS(inverse(Mat2{1, 2, 2, 4}));
}

TEST(matrix_bad_initializer_throws) {
    CHECK_THROWS((Mat2{1, 2, 3}));
}

TEST(wrap_angle_range) {
    CHECK_NEAR(wrapAngle(0.0), 0.0, 1e-12);
    CHECK_NEAR(wrapAngle(kPi), kPi, 1e-12);
    CHECK_NEAR(wrapAngle(-kPi), kPi, 1e-12);
    CHECK_NEAR(wrapAngle(3.0 * kPi / 2.0), -kPi / 2.0, 1e-12);
    CHECK_NEAR(wrapAngle(-5.0 * kPi / 2.0), -kPi / 2.0, 1e-12);
}
