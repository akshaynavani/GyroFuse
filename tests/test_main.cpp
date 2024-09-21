#include "test_main.hpp"

#include <cstring>

int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : nullptr;
    int run = 0;
    int failedTests = 0;
    for (const auto& t : gftest::registry()) {
        if (filter && std::strstr(t.name.c_str(), filter) == nullptr) continue;
        const int before = gftest::failures();
        try {
            t.fn();
        } catch (const std::exception& e) {
            gftest::fail(t.name.c_str(), 0, std::string("unexpected exception: ") + e.what());
        }
        const bool ok = gftest::failures() == before;
        std::cout << (ok ? "[ PASS ] " : "[ FAIL ] ") << t.name << "\n";
        failedTests += ok ? 0 : 1;
        ++run;
    }
    std::cout << run - failedTests << "/" << run << " tests passed\n";
    return failedTests == 0 ? 0 : 1;
}
