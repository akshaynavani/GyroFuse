#pragma once

#include <array>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>

namespace gyrofuse {

// Minimal fixed-size row-major matrix. Sized for small filter states, so all
// operations are naive O(n^3) and allocation free.
template <std::size_t R, std::size_t C>
class Matrix {
public:
    Matrix() { data_.fill(0.0); }

    Matrix(std::initializer_list<double> values) {
        if (values.size() != R * C) throw std::invalid_argument("Matrix: wrong initializer size");
        std::size_t i = 0;
        for (double v : values) data_[i++] = v;
    }

    static Matrix identity() {
        static_assert(R == C, "identity requires a square matrix");
        Matrix m;
        for (std::size_t i = 0; i < R; ++i) m(i, i) = 1.0;
        return m;
    }

    static constexpr std::size_t rows() { return R; }
    static constexpr std::size_t cols() { return C; }

    double& operator()(std::size_t r, std::size_t c) { return data_[r * C + c]; }
    double operator()(std::size_t r, std::size_t c) const { return data_[r * C + c]; }

    Matrix operator+(const Matrix& o) const {
        Matrix m;
        for (std::size_t i = 0; i < R * C; ++i) m.data_[i] = data_[i] + o.data_[i];
        return m;
    }

    Matrix operator-(const Matrix& o) const {
        Matrix m;
        for (std::size_t i = 0; i < R * C; ++i) m.data_[i] = data_[i] - o.data_[i];
        return m;
    }

    Matrix operator*(double s) const {
        Matrix m;
        for (std::size_t i = 0; i < R * C; ++i) m.data_[i] = data_[i] * s;
        return m;
    }

    template <std::size_t K>
    Matrix<R, K> operator*(const Matrix<C, K>& o) const {
        Matrix<R, K> m;
        for (std::size_t i = 0; i < R; ++i)
            for (std::size_t k = 0; k < K; ++k) {
                double acc = 0.0;
                for (std::size_t j = 0; j < C; ++j) acc += (*this)(i, j) * o(j, k);
                m(i, k) = acc;
            }
        return m;
    }

    Matrix<C, R> transpose() const {
        Matrix<C, R> m;
        for (std::size_t i = 0; i < R; ++i)
            for (std::size_t j = 0; j < C; ++j) m(j, i) = (*this)(i, j);
        return m;
    }

private:
    std::array<double, R * C> data_{};
};

using Mat2 = Matrix<2, 2>;
using Vec2 = Matrix<2, 1>;

// Closed-form 2x2 inverse; throws on a singular matrix.
inline Mat2 inverse(const Mat2& m) {
    const double det = m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
    if (det == 0.0) throw std::domain_error("inverse: singular 2x2 matrix");
    const double inv = 1.0 / det;
    return Mat2{m(1, 1) * inv, -m(0, 1) * inv, -m(1, 0) * inv, m(0, 0) * inv};
}

}  // namespace gyrofuse
