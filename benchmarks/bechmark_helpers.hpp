/**
 * @file
 * @author lkoppel
 * Helper functions for writing benchmarks
 */

#ifndef WAVE_GEOMETRY_BENCHMARK_HELPERS_HPP
#define WAVE_GEOMETRY_BENCHMARK_HELPERS_HPP

// Include test helper functions - we will use them for debug checks
#include "../test/test.hpp"

#ifndef NDEBUG
/** Call ASSERT_EQ only in debug mode */
#define DEBUG_ASSERT_EQ(a, b) ASSERT_EQ(a, b)
/** Call ASSERT_APPROX only in debug mode */
#define DEBUG_ASSERT_APPROX(a, b) ASSERT_APPROX(a, b)
/** Call ASSERT_APPROX_PREC only in debug mode */
#define DEBUG_ASSERT_APPROX_PREC(a, b) ASSERT_APPROX_PREC(a, b)
#else
#define DEBUG_ASSERT_EQ(a, b)
#define DEBUG_ASSERT_APPROX(a, b)
#define DEBUG_ASSERT_APPROX_PREC(a, b)
#endif

/** Return a vector of random objects of type T, using setRandom() */
template <typename T>
std::vector<T, Eigen::aligned_allocator<T>> randomMatrices(int N) {
    std::vector<T, Eigen::aligned_allocator<T>> v;
    v.reserve(N);
    for (auto i = N; i--;) {
        v.push_back(T::Random());
    }
    return v;
}

// BENCHMARK_MAIN() prepended with setting gtest flag
#define WAVE_BENCHMARK_MAIN()                                     \
    int main(int argc, char **argv) {                             \
        testing::InitGoogleTest(&argc, argv);                     \
        ::testing::GTEST_FLAG(throw_on_failure) = true;           \
        ::benchmark::Initialize(&argc, argv);                     \
                                                                  \
        if (::benchmark::ReportUnrecognizedArguments(argc, argv)) \
            return 1;                                             \
        ::benchmark::RunSpecifiedBenchmarks();                    \
    }

#endif  // WAVE_GEOMETRY_BENCHMARK_HELPERS_HPP
