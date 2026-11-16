#pragma once

// Minimal assertion helpers: each test binary returns non-zero on failure so
// ctest reports it, and prints every failed expectation with its location.

#include <cmath>
#include <cstdio>

namespace testing_detail {
inline int& failures() {
    static int n = 0;
    return n;
}
}  // namespace testing_detail

#define CHECK(cond)                                                                   \
    do {                                                                              \
        if (!(cond)) {                                                                \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
            ++testing_detail::failures();                                             \
        }                                                                             \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                          \
    do {                                                                               \
        const double va_ = (a), vb_ = (b);                                             \
        if (!(std::fabs(va_ - vb_) <= (tol))) {                                        \
            std::fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s=%.6f %s=%.6f tol=%g\n",  \
                         __FILE__, __LINE__, #a, va_, #b, vb_, static_cast<double>(tol)); \
            ++testing_detail::failures();                                              \
        }                                                                              \
    } while (0)

#define TEST_MAIN_RESULT()                                                    \
    (testing_detail::failures() == 0                                          \
         ? (std::printf("all checks passed\n"), 0)                            \
         : (std::fprintf(stderr, "%d check(s) failed\n", testing_detail::failures()), 1))
