
#include <benchmark/benchmark.h>

#include "wave/geometry/geometry.hpp"
#include "wave/geometry/debug.hpp"
#include "../bechmark_helpers.hpp"

/* Change to test with matrix or quaternion
 * benchmark's BENCHMARK_TEMPLATE_F is limited, so I don't know a more convenient way to
 * do this */
using RotationType = Eigen::Matrix3d;
// using RotationType = Eigen::Quaterniond;

template <int I>
struct FrameN;

using Eigen::Matrix3d;
using Eigen::Vector3d;

template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <class RotType>
class RotateChain : public benchmark::Fixture {
 protected:
    const int N = 1;
    // Use wave::RotationM just for the Random() method, which produces valid SO(3)
    using waveRot = wave::MatrixRotation<RotType>;
    const EigenVector<waveRot> R1 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R2 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R3 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R4 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R5 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R6 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R7 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R8 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R9 = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R10 = randomMatrices<waveRot>(N);
    const EigenVector<Vector3d> v1 = randomMatrices<Vector3d>(N);
};

BENCHMARK_TEMPLATE_F(RotateChain, Hand1, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            // Evaluate result
            auto v2 = Vector3d{R1[i].value() * v1[i]};

            // Jacobian wrt rotation
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{R1[i].value()};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}


BENCHMARK_TEMPLATE_F(RotateChain, Hand2, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            // Jacobian wrt vector
            auto Jv1 = Matrix3d{R1[i].value() * R2[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * R1[i].value()};


            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}


BENCHMARK_TEMPLATE_F(RotateChain, Hand3, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            const auto &t1 = R1[i].value();
            auto t2 = RotationType{t1 * R2[i].value()};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{t2 * R3[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * t1};
            auto JR3 = Matrix3d{wave::crossMatrix(-v2) * t2};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(JR3);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}


BENCHMARK_TEMPLATE_F(RotateChain, Hand4, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            const auto &t1 = R1[i].value();
            auto t2 = RotationType{t1 * R2[i].value()};
            auto t3 = RotationType{t2 * R3[i].value()};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{t3 * R4[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * t1};
            auto JR3 = Matrix3d{wave::crossMatrix(-v2) * t2};
            auto JR4 = Matrix3d{wave::crossMatrix(-v2) * t3};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(JR3);
            benchmark::DoNotOptimize(JR4);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}

BENCHMARK_TEMPLATE_F(RotateChain, Hand5, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            const auto &t1 = R1[i].value();
            auto t2 = RotationType{t1 * R2[i].value()};
            auto t3 = RotationType{t2 * R3[i].value()};
            auto t4 = RotationType{t3 * R4[i].value()};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{t4 * R5[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * t1};
            auto JR3 = Matrix3d{wave::crossMatrix(-v2) * t2};
            auto JR4 = Matrix3d{wave::crossMatrix(-v2) * t3};
            auto JR5 = Matrix3d{wave::crossMatrix(-v2) * t4};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(JR3);
            benchmark::DoNotOptimize(JR4);
            benchmark::DoNotOptimize(JR5);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}

BENCHMARK_TEMPLATE_F(RotateChain, Hand6, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            const auto &t1 = R1[i].value();
            auto t2 = RotationType{t1 * R2[i].value()};
            auto t3 = RotationType{t2 * R3[i].value()};
            auto t4 = RotationType{t3 * R4[i].value()};
            auto t5 = RotationType{t4 * R5[i].value()};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{t5 * R6[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * t1};
            auto JR3 = Matrix3d{wave::crossMatrix(-v2) * t2};
            auto JR4 = Matrix3d{wave::crossMatrix(-v2) * t3};
            auto JR5 = Matrix3d{wave::crossMatrix(-v2) * t4};
            auto JR6 = Matrix3d{wave::crossMatrix(-v2) * t5};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(JR3);
            benchmark::DoNotOptimize(JR4);
            benchmark::DoNotOptimize(JR5);
            benchmark::DoNotOptimize(JR6);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}

BENCHMARK_TEMPLATE_F(RotateChain, Hand7, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            const auto &t1 = R1[i].value();
            auto t2 = RotationType{t1 * R2[i].value()};
            auto t3 = RotationType{t2 * R3[i].value()};
            auto t4 = RotationType{t3 * R4[i].value()};
            auto t5 = RotationType{t4 * R5[i].value()};
            auto t6 = RotationType{t5 * R6[i].value()};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{t6 * R7[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * t1};
            auto JR3 = Matrix3d{wave::crossMatrix(-v2) * t2};
            auto JR4 = Matrix3d{wave::crossMatrix(-v2) * t3};
            auto JR5 = Matrix3d{wave::crossMatrix(-v2) * t4};
            auto JR6 = Matrix3d{wave::crossMatrix(-v2) * t5};
            auto JR7 = Matrix3d{wave::crossMatrix(-v2) * t6};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(JR3);
            benchmark::DoNotOptimize(JR4);
            benchmark::DoNotOptimize(JR5);
            benchmark::DoNotOptimize(JR6);
            benchmark::DoNotOptimize(JR7);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}

BENCHMARK_TEMPLATE_F(RotateChain, Hand8, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            const auto &t1 = R1[i].value();
            auto t2 = RotationType{t1 * R2[i].value()};
            auto t3 = RotationType{t2 * R3[i].value()};
            auto t4 = RotationType{t3 * R4[i].value()};
            auto t5 = RotationType{t4 * R5[i].value()};
            auto t6 = RotationType{t5 * R6[i].value()};
            auto t7 = RotationType{t6 * R7[i].value()};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{t7 * R8[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * t1};
            auto JR3 = Matrix3d{wave::crossMatrix(-v2) * t2};
            auto JR4 = Matrix3d{wave::crossMatrix(-v2) * t3};
            auto JR5 = Matrix3d{wave::crossMatrix(-v2) * t4};
            auto JR6 = Matrix3d{wave::crossMatrix(-v2) * t5};
            auto JR7 = Matrix3d{wave::crossMatrix(-v2) * t6};
            auto JR8 = Matrix3d{wave::crossMatrix(-v2) * t7};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(JR3);
            benchmark::DoNotOptimize(JR4);
            benchmark::DoNotOptimize(JR5);
            benchmark::DoNotOptimize(JR6);
            benchmark::DoNotOptimize(JR7);
            benchmark::DoNotOptimize(JR8);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}

BENCHMARK_TEMPLATE_F(RotateChain, Hand9, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            const auto &t1 = R1[i].value();
            auto t2 = RotationType{t1 * R2[i].value()};
            auto t3 = RotationType{t2 * R3[i].value()};
            auto t4 = RotationType{t3 * R4[i].value()};
            auto t5 = RotationType{t4 * R5[i].value()};
            auto t6 = RotationType{t5 * R6[i].value()};
            auto t7 = RotationType{t6 * R7[i].value()};
            auto t8 = RotationType{t7 * R8[i].value()};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{t8 * R9[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * t1};
            auto JR3 = Matrix3d{wave::crossMatrix(-v2) * t2};
            auto JR4 = Matrix3d{wave::crossMatrix(-v2) * t3};
            auto JR5 = Matrix3d{wave::crossMatrix(-v2) * t4};
            auto JR6 = Matrix3d{wave::crossMatrix(-v2) * t5};
            auto JR7 = Matrix3d{wave::crossMatrix(-v2) * t6};
            auto JR8 = Matrix3d{wave::crossMatrix(-v2) * t7};
            auto JR9 = Matrix3d{wave::crossMatrix(-v2) * t8};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(JR3);
            benchmark::DoNotOptimize(JR4);
            benchmark::DoNotOptimize(JR5);
            benchmark::DoNotOptimize(JR6);
            benchmark::DoNotOptimize(JR7);
            benchmark::DoNotOptimize(JR8);
            benchmark::DoNotOptimize(JR9);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}

BENCHMARK_TEMPLATE_F(RotateChain, Hand10, RotationType)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            const auto &t1 = R1[i].value();
            auto t2 = RotationType{t1 * R2[i].value()};
            auto t3 = RotationType{t2 * R3[i].value()};
            auto t4 = RotationType{t3 * R4[i].value()};
            auto t5 = RotationType{t4 * R5[i].value()};
            auto t6 = RotationType{t5 * R6[i].value()};
            auto t7 = RotationType{t6 * R7[i].value()};
            auto t8 = RotationType{t7 * R8[i].value()};
            auto t9 = RotationType{t8 * R9[i].value()};

            // Jacobian wrt vector
            auto Jv1 = Matrix3d{t9 * R10[i].value()};

            // Result
            auto v2 = Vector3d{Jv1 * v1[i]};

            // Jacobian wrt rotations
            auto JR1 = Matrix3d{wave::crossMatrix(-v2)};
            auto JR2 = Matrix3d{wave::crossMatrix(-v2) * t1};
            auto JR3 = Matrix3d{wave::crossMatrix(-v2) * t2};
            auto JR4 = Matrix3d{wave::crossMatrix(-v2) * t3};
            auto JR5 = Matrix3d{wave::crossMatrix(-v2) * t4};
            auto JR6 = Matrix3d{wave::crossMatrix(-v2) * t5};
            auto JR7 = Matrix3d{wave::crossMatrix(-v2) * t6};
            auto JR8 = Matrix3d{wave::crossMatrix(-v2) * t7};
            auto JR9 = Matrix3d{wave::crossMatrix(-v2) * t8};
            auto JR10 = Matrix3d{wave::crossMatrix(-v2) * t9};

            benchmark::DoNotOptimize(v2);
            benchmark::DoNotOptimize(JR1);
            benchmark::DoNotOptimize(JR2);
            benchmark::DoNotOptimize(JR3);
            benchmark::DoNotOptimize(JR4);
            benchmark::DoNotOptimize(JR5);
            benchmark::DoNotOptimize(JR6);
            benchmark::DoNotOptimize(JR7);
            benchmark::DoNotOptimize(JR8);
            benchmark::DoNotOptimize(JR9);
            benchmark::DoNotOptimize(JR10);
            benchmark::DoNotOptimize(Jv1);
        }
    }
}

WAVE_BENCHMARK_MAIN()
