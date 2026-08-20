/* bcn_test.h — the whole test harness, on purpose.
 *
 * DEVIATION FROM T013, recorded deliberately: tasks.md says "Register GoogleTest + CTest". The tests are
 * registered with CTest exactly as asked, but they are plain C rather than GoogleTest, because tasks.md
 * also names every test file `.c` and a C translation unit cannot include gtest.h. Picking C over C++
 * resolves that in the direction the architecture already points: core/ is zero-dependency C11 so that it
 * builds on the Pi and cross-compiles from WSL2 with nothing but a cross-gcc (plan.md §Structure
 * Decision). Requiring GoogleTest — C++, plus a network FetchContent — to test it would put a dependency
 * in front of exactly the paths the core/shell split exists to keep dependency-free. GoogleTest remains
 * the harness for the autoc side and for any future C++ test of io/.
 */
#ifndef BCN_TEST_H
#define BCN_TEST_H

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static int bcn_t_fail = 0;
static int bcn_t_ran  = 0;

#define CHECK(cond, ...)                                                          \
    do {                                                                          \
        bcn_t_ran++;                                                              \
        if (!(cond)) {                                                            \
            bcn_t_fail++;                                                         \
            fprintf(stderr, "FAIL %s:%d: %s\n      ", __FILE__, __LINE__, #cond); \
            fprintf(stderr, __VA_ARGS__);                                         \
            fprintf(stderr, "\n");                                                \
        }                                                                         \
    } while (0)

#define CHECK_EQ_U(a, b) CHECK((uint64_t)(a) == (uint64_t)(b), \
    "expected %" PRIu64 ", got %" PRIu64, (uint64_t)(b), (uint64_t)(a))

#define CHECK_EQ_I(a, b) CHECK((int64_t)(a) == (int64_t)(b), \
    "expected %" PRId64 ", got %" PRId64, (int64_t)(b), (int64_t)(a))

#define CHECK_STR_HAS(hay, needle) CHECK(strstr((hay), (needle)) != NULL, \
    "expected message to contain \"%s\", got \"%s\"", (needle), (hay))

#define BCN_TEST_MAIN_END()                                                    \
    do {                                                                       \
        if (bcn_t_fail) {                                                      \
            fprintf(stderr, "\n%d/%d checks FAILED\n", bcn_t_fail, bcn_t_ran); \
            return 1;                                                          \
        }                                                                      \
        printf("%d checks passed\n", bcn_t_ran);                               \
        return 0;                                                              \
    } while (0)

#endif /* BCN_TEST_H */
