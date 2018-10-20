#include <benchmark/benchmark.h>

#include "wave/geometry/geometry.hpp"
#include "wave/geometry/debug.hpp"
#include "../bechmark_helpers.hpp"

template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

class RotateChain : public benchmark::Fixture {
 protected:
    const int N = 1;
    const EigenVector<wave::RotationMd> R1 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R2 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R3 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R4 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R5 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R6 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R7 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R8 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R9 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::RotationMd> R10 = randomMatrices<wave::RotationMd>(N);
    const EigenVector<wave::Translationd> v10 = randomMatrices<wave::Translationd>(N);
};

BENCHMARK_F(RotateChain, waveUntyped1)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            // use internal::evaluateWithJacobians to force untyped evaluator despite
            // unique types
            auto [v0, J10, Jv10] =
              wave::internal::evaluateWithJacobians(R10[i] * v10[i], R10[i], v10[i]);

            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped2)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J9, J10, Jv10] =
              (R9[i] * R10[i] * v10[i]).evalWithJacobians(R9[i], R10[i], v10[i]);

            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped3)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J8, J9, J10, Jv10] =
              (R8[i] * R9[i] * R10[i] * v10[i])
                .evalWithJacobians(R8[i], R9[i], R10[i], v10[i]);

            benchmark::DoNotOptimize(J8);
            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped4)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J7, J8, J9, J10, Jv10] =
              (R7[i] * R8[i] * R9[i] * R10[i] * v10[i])
                .evalWithJacobians(R7[i], R8[i], R9[i], R10[i], v10[i]);

            benchmark::DoNotOptimize(J7);
            benchmark::DoNotOptimize(J8);
            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped5)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J6, J7, J8, J9, J10, Jv10] =
              (R6[i] * R7[i] * R8[i] * R9[i] * R10[i] * v10[i])
                .evalWithJacobians(R6[i], R7[i], R8[i], R9[i], R10[i], v10[i]);

            benchmark::DoNotOptimize(J6);
            benchmark::DoNotOptimize(J7);
            benchmark::DoNotOptimize(J8);
            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped6)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J5, J6, J7, J8, J9, J10, Jv10] =
              (R5[i] * R6[i] * R7[i] * R8[i] * R9[i] * R10[i] * v10[i])
                .evalWithJacobians(R5[i], R6[i], R7[i], R8[i], R9[i], R10[i], v10[i]);
            benchmark::DoNotOptimize(J5);
            benchmark::DoNotOptimize(J6);
            benchmark::DoNotOptimize(J7);
            benchmark::DoNotOptimize(J8);
            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped7)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J4, J5, J6, J7, J8, J9, J10, Jv10] =
              (R4[i] * R5[i] * R6[i] * R7[i] * R8[i] * R9[i] * R10[i] * v10[i])
                .evalWithJacobians(
                  R4[i], R5[i], R6[i], R7[i], R8[i], R9[i], R10[i], v10[i]);
            benchmark::DoNotOptimize(J4);
            benchmark::DoNotOptimize(J5);
            benchmark::DoNotOptimize(J6);
            benchmark::DoNotOptimize(J7);
            benchmark::DoNotOptimize(J8);
            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped8)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J3, J4, J5, J6, J7, J8, J9, J10, Jv10] =
              (R3[i] * R4[i] * R5[i] * R6[i] * R7[i] * R8[i] * R9[i] * R10[i] * v10[i])
                .evalWithJacobians(
                  R3[i], R4[i], R5[i], R6[i], R7[i], R8[i], R9[i], R10[i], v10[i]);
            benchmark::DoNotOptimize(J3);
            benchmark::DoNotOptimize(J4);
            benchmark::DoNotOptimize(J5);
            benchmark::DoNotOptimize(J6);
            benchmark::DoNotOptimize(J7);
            benchmark::DoNotOptimize(J8);
            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped9)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J2, J3, J4, J5, J6, J7, J8, J9, J10, Jv10] =
              (R2[i] * R3[i] * R4[i] * R5[i] * R6[i] * R7[i] * R8[i] * R9[i] * R10[i] *
               v10[i])
                .evalWithJacobians(
                  R2[i], R3[i], R4[i], R5[i], R6[i], R7[i], R8[i], R9[i], R10[i], v10[i]);

            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J3);
            benchmark::DoNotOptimize(J4);
            benchmark::DoNotOptimize(J5);
            benchmark::DoNotOptimize(J6);
            benchmark::DoNotOptimize(J7);
            benchmark::DoNotOptimize(J8);
            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, waveUntyped10)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto [v0, J1, J2, J3, J4, J5, J6, J7, J8, J9, J10, Jv10] =
              (R1[i] * R2[i] * R3[i] * R4[i] * R5[i] * R6[i] * R7[i] * R8[i] * R9[i] *
               R10[i] * v10[i])
                .evalWithJacobians(R1[i],
                                   R2[i],
                                   R3[i],
                                   R4[i],
                                   R5[i],
                                   R6[i],
                                   R7[i],
                                   R8[i],
                                   R9[i],
                                   R10[i],
                                   v10[i]);

            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J3);
            benchmark::DoNotOptimize(J4);
            benchmark::DoNotOptimize(J5);
            benchmark::DoNotOptimize(J6);
            benchmark::DoNotOptimize(J7);
            benchmark::DoNotOptimize(J8);
            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

WAVE_BENCHMARK_MAIN()
