#include "tsim_test.h"

int main() {
    int failing_cases = 0;
    for (const tsim_test::Case &test_case : tsim_test::registry()) {
        const int before = tsim_test::failure_count();
        test_case.fn();
        const bool passed = tsim_test::failure_count() == before;
        std::printf("[%s] %s\n", passed ? " ok " : "FAIL", test_case.name);
        if (!passed) {
            ++failing_cases;
        }
    }
    std::printf("%zu cases, %d checks, %d failing cases\n", tsim_test::registry().size(),
                tsim_test::check_count(), failing_cases);
    return (failing_cases == 0) ? 0 : 1;
}
