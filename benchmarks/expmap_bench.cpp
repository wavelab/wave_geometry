#include <benchmark/benchmark.h>
#include <wave/geometry/geometry.hpp>
#include "bechmark_helpers.hpp"

/** Return a vector of random objects of type T, using setRandom() */
template <typename T>
std::vector<T, Eigen::aligned_allocator<T>> randomSmallVectors(int N, double norm) {
    std::vector<T, Eigen::aligned_allocator<T>> v(N);
    for (auto i = N; i--;) {
        v[i] = T::Random();
        v[i] /= v[i].norm();
        v[i] *= norm;
    }
    return v;
}

using Scalar = double;

using Mat3 = Eigen::Matrix<Scalar, 3, 3>;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Quat = Eigen::Quaternion<Scalar>;
using wave::crossMatrix;

inline Mat3 expMapM(const Vec3 &v) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    // Rodrigues formula - see http://ethaneade.com/lie.pdf
    const auto angle2 = v.squaredNorm();
    const auto angle = sqrt(angle2);

    if (angle2 > Eigen::NumTraits<Scalar>::epsilon()) {
        const auto s = sin(angle);
        const auto s_half = sin(angle / 2);

        return Mat3{Mat3::Identity() + s / angle * crossMatrix(v) +
                    2 * s_half * s_half / angle2 * crossMatrix(v) * crossMatrix(v)};
    } else {
        // Small angle: use linear terms of Taylor expansion
        return Mat3{Mat3::Identity() + crossMatrix(v)};
    }
}

inline Quat expMapQ(const Vec3 &v) {
    return wave::quaternionFromExpMap(v);
}

inline Mat3 expMapQM(const Vec3 &v) {
    return expMapQ(v).matrix();
}

template <Quat (*Fn)(const Vec3 &)>
inline void BM_expMapQFn(benchmark::State &state) {
    const auto N = 100;
    const auto norm = std::pow(10.0, state.range(0));
    const auto vs = randomSmallVectors<Vec3>(N, norm);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            const Quat result = Fn(vs[i]);

            benchmark::DoNotOptimize(result);
            DEBUG_ASSERT_APPROX(expMapM(vs[i]), result.matrix());
        }
    }
}

template <Mat3 (*Fn)(const Vec3 &)>
inline void BM_expMapFn(benchmark::State &state) {
    const auto N = 100;
    const auto norm = std::pow(10.0, state.range(0));
    const auto vs = randomSmallVectors<Vec3>(N, norm);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            const Mat3 result = Fn(vs[i]);

            benchmark::DoNotOptimize(result);
            DEBUG_ASSERT_APPROX(expMapM(vs[i]), result);
        }
    }
}


// Powers of 10 to use as the norms of the input rotation vectors
const int lopow = -1;
const int hipow = -1;

BENCHMARK_TEMPLATE(BM_expMapQFn, expMapQ)->Range(lopow, hipow);
BENCHMARK_TEMPLATE(BM_expMapFn, expMapQM)->Range(lopow, hipow);
BENCHMARK_TEMPLATE(BM_expMapFn, expMapM)->Range(lopow, hipow);

WAVE_BENCHMARK_MAIN()
