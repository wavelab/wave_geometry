#include <benchmark/benchmark.h>

#include "../bechmark_helpers.hpp"
#include "wave/geometry/dynamic.hpp"
#include "wave/geometry/geometry.hpp"

template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

void BM_waveAll(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);
    // Produce the expression tree
    //    EigenVector<wave::Proxy<wave::RotationMd>> proxies;
    auto expr = makeProxy(wave::Translationd::Random());
    for (auto i = N; i > 0; --i) {
        expr = makeProxy(makeProxy(wave::RotationMd::Random()) * expr);
    }

    for (auto _ : state) {
        auto [res, jac_map] = wave::internal::evaluateWithDynamicReverseJacobians(expr);

        benchmark::DoNotOptimize(res);
        benchmark::DoNotOptimize(jac_map);
    }
}

void BM_dynamicNoVirtual(benchmark::State &state) {
    auto v = wave::Translationd::Random();
    std::array<wave::RotationMd, 10> R{};
    for (int i = 0; i < 10; ++i) {
        R[i] = wave::RotationMd::Random();
    }

    for (auto _ : state) {
        auto expr =
          R[0] * R[1] * R[2] * R[3] * R[4] * R[5] * R[6] * R[7] * R[8] * R[9] * v;

        auto [res, jac_map] = wave::internal::evaluateWithDynamicReverseJacobians(expr);

        benchmark::DoNotOptimize(res);
        benchmark::DoNotOptimize(jac_map);
    }
}


void BM_waveDynamicLeaves(benchmark::State &state) {
    const auto N = state.range(0);
    // Produce the expression tree
    EigenVector<wave::Proxy<wave::RotationMd>> proxies;
    auto expr = makeProxy(wave::Translationd::Random());
    for (auto i = N; i > 0; --i) {
        proxies.emplace_back(wave::RotationMd::Random());
        expr = makeProxy(proxies.back() * expr);
    }

    for (auto _ : state) {
        auto map = wave::internal::getLeavesMap(expr);
        benchmark::DoNotOptimize(map);
    }
}

// BENCHMARK(BM_waveDynamicLeaves)->Range(10, 200000)->Complexity();
// BENCHMARK(BM_waveDynamic)->Arg(10);
BENCHMARK(BM_waveAll)->RangeMultiplier(2)->DenseRange(1, 1 << 14)->Complexity();
// BENCHMARK(BM_dynamicNoVirtual);

WAVE_BENCHMARK_MAIN()
