#include <benchmark/benchmark.h>
#include "wave/geometry/src/util/math/IdentityMatrix.hpp"
#include "bechmark_helpers.hpp"

void BM_Eigen_IdentityMultiply1(benchmark::State &state) {
    const auto N = state.range(0);
    const auto a = randomMatrices<Eigen::Matrix3d>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            const Eigen::Matrix3d result = Eigen::Matrix3d::Identity() * a[i];

            benchmark::DoNotOptimize(result.data());
            DEBUG_ASSERT_APPROX(a[i], result);
        }
    }
}

void BM_wave_IdentityMultiply1(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Eigen::Matrix3d>(N);

    using Identity = wave::IdentityMatrix<double, 3>;

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Matrix3d result = Identity{} * a[i];

            benchmark::DoNotOptimize(result.data());
            DEBUG_ASSERT_APPROX(result, a[i]);
        }
    }
}

void BM_Eigen_IdentityMultiply6(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Eigen::Matrix3d>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Matrix3d result =
              Eigen::Matrix3d::Identity() * Eigen::Matrix3d::Identity() *
              Eigen::Matrix3d::Identity() * a[i] * Eigen::Matrix3d::Identity() *
              Eigen::Matrix3d::Identity() * Eigen::Matrix3d::Identity();

            benchmark::DoNotOptimize(result.data());
            DEBUG_ASSERT_APPROX(result, a[i]);
        }
    }
}

void BM_wave_IdentityMultiply6(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Eigen::Matrix3d>(N);

    using Identity = wave::IdentityMatrix<double, 3>;

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Matrix3d result = Identity{} * Identity{} * Identity{} * a[i] *
                                     Identity{} * Identity{} * Identity{};

            benchmark::DoNotOptimize(result.data());
            DEBUG_ASSERT_APPROX(result, a[i]);
        }
    }
}

void BM_wave_IdentityMultiply7(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Eigen::Matrix3d>(N);

    using Identity = wave::IdentityMatrix<double, 3>;

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Matrix3d result = 4.6 * Identity{} * Identity{} * Identity{} * a[i] *
                                     Identity{} * Identity{} * Identity{} * Identity{};

            benchmark::DoNotOptimize(result.data());
            DEBUG_ASSERT_APPROX(result, 4.6 * a[i]);
        }
    }
}

const int reps = 1000;

BENCHMARK(BM_Eigen_IdentityMultiply1)->Arg(reps);
BENCHMARK(BM_wave_IdentityMultiply1)->Arg(reps);
BENCHMARK(BM_Eigen_IdentityMultiply6)->Arg(reps);
BENCHMARK(BM_wave_IdentityMultiply6)->Arg(reps);
BENCHMARK(BM_wave_IdentityMultiply7)->Arg(reps);

WAVE_BENCHMARK_MAIN()
