#include <benchmark/benchmark.h>
#include <wave/geometry/geometry.hpp>
#include "../bechmark_helpers.hpp"

using Scalar = double;

using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Quat = Eigen::Quaternion<Scalar>;
using wave::crossMatrix;
const int N = 100;

inline void BM_logMapQ(benchmark::State &state) {
    const auto wave_qs = randomMatrices<wave::RotationQd>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            const Vec3 result = wave::rotationVectorFromQuaternion(wave_qs[i].value());

            benchmark::DoNotOptimize(result);
            DEBUG_ASSERT_APPROX_PREC(
              wave::rotationVectorFromMatrix(wave_qs[i].value().matrix()), result, 1e-7);
        }
    }
}

inline void BM_logMapM(benchmark::State &state) {
    const auto wave_ms = randomMatrices<wave::RotationMd>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            const Vec3 result = wave::rotationVectorFromMatrix(wave_ms[i].value());

            benchmark::DoNotOptimize(result);
        }
    }
}

inline void BM_logMapQM(benchmark::State &state) {
    const auto wave_ms = randomMatrices<wave::RotationMd>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            const Vec3 result =
              wave::rotationVectorFromQuaternion(Quat{wave_ms[i].value()});

            benchmark::DoNotOptimize(result);
        }
    }
}

inline void BM_logMapMQ(benchmark::State &state) {
    const auto wave_qs = randomMatrices<wave::RotationQd>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            const Vec3 result =
              wave::rotationVectorFromMatrix(wave_qs[i].value().matrix());

            benchmark::DoNotOptimize(result);
        }
    }
}


BENCHMARK(BM_logMapQ);
BENCHMARK(BM_logMapM);
BENCHMARK(BM_logMapQM);
BENCHMARK(BM_logMapMQ);

WAVE_BENCHMARK_MAIN()
