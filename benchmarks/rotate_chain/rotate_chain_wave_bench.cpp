#include <benchmark/benchmark.h>

#include "wave/geometry/geometry.hpp"
#include "wave/geometry/debug.hpp"
#include "../bechmark_helpers.hpp"

// #define RotationMFd RotationQFd

template <int I>
struct FrameN;

template <int I, int J>
using RMFd = wave::RotationMFd<FrameN<I>, FrameN<J>>;

template <int I, int J, int K>
using TFd = wave::TranslationFd<FrameN<I>, FrameN<J>, FrameN<K>>;


template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <int I, int J>
using RMFdVector = EigenVector<RMFd<I, J>>;


template <int I, int J>
RMFd<I, J> randomRMFd() {
    return RMFd<I, J>::Random();
};

class RotateChain : public benchmark::Fixture {
 protected:
    const int N = 1;
    const RMFdVector<0, 1> R1 = randomMatrices<RMFd<0, 1>>(N);
    const RMFdVector<1, 2> R2 = randomMatrices<RMFd<1, 2>>(N);
    const RMFdVector<2, 3> R3 = randomMatrices<RMFd<2, 3>>(N);
    const RMFdVector<3, 4> R4 = randomMatrices<RMFd<3, 4>>(N);
    const RMFdVector<4, 5> R5 = randomMatrices<RMFd<4, 5>>(N);
    const RMFdVector<5, 6> R6 = randomMatrices<RMFd<5, 6>>(N);
    const RMFdVector<6, 7> R7 = randomMatrices<RMFd<6, 7>>(N);
    const RMFdVector<7, 8> R8 = randomMatrices<RMFd<7, 8>>(N);
    const RMFdVector<8, 9> R9 = randomMatrices<RMFd<8, 9>>(N);
    const RMFdVector<9, 10> R10 = randomMatrices<RMFd<9, 10>>(N);
    const EigenVector<TFd<10, 0, 1>> v10 = randomMatrices<TFd<10, 0, 1>>(N);
};

BENCHMARK_F(RotateChain, wave1)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J10, Jv10] = (R10[i] * v10[i]).evalWithJacobians(R10[i], v10[i]);

            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, wave2)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J9, J10, Jv10] =
              (R9[i] * R10[i] * v10[i]).evalWithJacobians(R9[i], R10[i], v10[i]);

            benchmark::DoNotOptimize(J9);
            benchmark::DoNotOptimize(J10);
            benchmark::DoNotOptimize(Jv10);
            benchmark::DoNotOptimize(v0);
        }
    }
}

BENCHMARK_F(RotateChain, wave3)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J8, J9, J10, Jv10] =
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

BENCHMARK_F(RotateChain, wave4)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J7, J8, J9, J10, Jv10] =
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

BENCHMARK_F(RotateChain, wave5)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J6, J7, J8, J9, J10, Jv10] =
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

BENCHMARK_F(RotateChain, wave6)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J5, J6, J7, J8, J9, J10, Jv10] =
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

BENCHMARK_F(RotateChain, wave7)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J4, J5, J6, J7, J8, J9, J10, Jv10] =
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

BENCHMARK_F(RotateChain, wave8)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J3, J4, J5, J6, J7, J8, J9, J10, Jv10] =
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

BENCHMARK_F(RotateChain, wave9)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J2, J3, J4, J5, J6, J7, J8, J9, J10, Jv10] =
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

BENCHMARK_F(RotateChain, wave10)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            auto[v0, J1, J2, J3, J4, J5, J6, J7, J8, J9, J10, Jv10] =
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
