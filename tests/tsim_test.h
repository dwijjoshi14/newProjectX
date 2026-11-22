/* tsim_test.h, a small in repo test framework with no dependencies. A test is
 * a static function registered at load time. Checks count, failures print.
 */
#ifndef TSIM_TEST_H
#define TSIM_TEST_H

#include <cstdio>
#include <vector>

namespace tsim_test {

struct Case {
    const char *name;
    void (*fn)();
};

inline std::vector<Case> &registry() {
    static std::vector<Case> cases;
    return cases;
}

inline int &check_count() {
    static int count = 0;
    return count;
}

inline int &failure_count() {
    static int count = 0;
    return count;
}

inline void report_failure(const char *file, int line, const char *expression) {
    std::fprintf(stderr, "%s:%d: check failed: %s\n", file, line, expression);
    ++failure_count();
}

struct Registrar {
    Registrar(const char *name, void (*fn)()) {
        registry().push_back(Case{name, fn});
    }
};

} // namespace tsim_test

#define TSIM_TEST(name)                                                                            \
    static void name();                                                                            \
    static const tsim_test::Registrar ts1m_registrar_##name(#name, name);                          \
    static void name()

#define TSIM_CHECK(expression)                                                                     \
    do {                                                                                           \
        ++tsim_test::check_count();                                                                \
        if (!(expression)) {                                                                       \
            tsim_test::report_failure(__FILE__, __LINE__, #expression);                            \
        }                                                                                          \
    } while (0)

/* Compare against a relative tolerance. Expected values near zero use an
 * absolute tolerance of the same size, which is stated at the call site. */
#define TSIM_CHECK_NEAR(actual, expected, relative_tolerance)                                      \
    do {                                                                                           \
        ++tsim_test::check_count();                                                                \
        const double actual_v = (actual);                                                          \
        const double expected_v = (expected);                                                      \
        const double tolerance_v = (relative_tolerance);                                           \
        const double difference_v =                                                                \
            (actual_v > expected_v) ? (actual_v - expected_v) : (expected_v - actual_v);           \
        const double scale_v = (expected_v < 0.0) ? -expected_v : expected_v;                      \
        const double allowed_v = tolerance_v * ((scale_v > 1.0) ? scale_v : 1.0);                  \
        if (!(difference_v <= allowed_v)) {                                                        \
            tsim_test::report_failure(__FILE__, __LINE__, #actual " vs " #expected);               \
            std::fprintf(stderr, "  actual %.12g expected %.12g allowed %.3g\n", actual_v,         \
                         expected_v, allowed_v);                                                   \
        }                                                                                          \
    } while (0)

#endif /* TSIM_TEST_H */
