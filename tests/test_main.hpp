#pragma once
// Tiny dependency-free test harness: TEST() self-registers, CHECK* record failures.

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace gftest {

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

inline int& failures() {
    static int count = 0;
    return count;
}

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline void fail(const char* file, int line, const std::string& msg) {
    ++failures();
    std::cerr << "  " << file << ":" << line << ": " << msg << "\n";
}

}  // namespace gftest

#define GF_CONCAT_INNER(a, b) a##b
#define GF_CONCAT(a, b) GF_CONCAT_INNER(a, b)

#define TEST(name)                                                              \
    static void name();                                                         \
    static ::gftest::Registrar GF_CONCAT(registrar_, name)(#name, &name);       \
    static void name()

#define CHECK(cond)                                                             \
    do {                                                                        \
        if (!(cond)) ::gftest::fail(__FILE__, __LINE__, "CHECK(" #cond ") failed"); \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                   \
    do {                                                                        \
        const double gf_a = (a), gf_b = (b);                                    \
        if (!(std::fabs(gf_a - gf_b) <= (tol)))                                 \
            ::gftest::fail(__FILE__, __LINE__,                                  \
                           std::string(#a " = ") + std::to_string(gf_a) +       \
                               ", expected " + std::to_string(gf_b) + " +- " +  \
                               std::to_string(tol));                            \
    } while (0)

#define CHECK_THROWS(expr)                                                      \
    do {                                                                        \
        bool gf_threw = false;                                                  \
        try { (void)(expr); } catch (...) { gf_threw = true; }                  \
        if (!gf_threw) ::gftest::fail(__FILE__, __LINE__, "expected exception: " #expr); \
    } while (0)
