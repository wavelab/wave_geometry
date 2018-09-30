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

Mat3 expMapM(const Vec3 &v) {
    using std::sin;
    using std::cos;
    using std::sqrt;
    // Rodrigues formula - see http://ethaneade.com/lie.pdf
    const auto angle2 = v.squaredNorm();
    const auto angle = sqrt(angle2);
    if (angle2 * angle2 > Eigen::NumTraits<Scalar>::epsilon()) {
        return Mat3{Mat3::Identity() + sin(angle) / angle * crossMatrix(v) +
                    (Scalar{1} - cos(angle)) / angle2 * crossMatrix(v) * crossMatrix(v)};
    } else {
        // Small angle: use Taylor expansions
        return Mat3{Mat3::Identity() + crossMatrix(v) +
                    Scalar{0.5} * crossMatrix(v) * crossMatrix(v)};
    }
}

Mat3 expMapMNoCheck(const Vec3 &v) {
    using std::sin;
    using std::cos;
    using std::sqrt;
    // Rodrigues formula - see http://ethaneade.com/lie.pdf
    const auto angle2 = v.squaredNorm();
    const auto angle = sqrt(angle2);
    return Mat3{Mat3::Identity() + sin(angle) / angle * crossMatrix(v) +
                (Scalar{1} - cos(angle)) / angle2 * crossMatrix(v) * crossMatrix(v)};
}

Mat3 expMapQMNoCheck(const Vec3 &v) {
    using std::sin;
    using std::cos;
    const Scalar angle = v.norm();
    const Scalar s = sin(angle / 2) / angle;
    const Scalar c = cos(angle / 2);
    // storage order x, y, z, w
    Quat q;
    q.coeffs() << s * v, c;
    return q.matrix();
}

Mat3 expMapQM(const Vec3 &v) {
    Quat q;

    using std::sin;
    using std::cos;
    using std::sqrt;
    const Scalar angle2 = v.squaredNorm();
    const Scalar angle = sqrt(angle2);
    Scalar s;
    Scalar c;

    if (angle2 * angle2 > Eigen::NumTraits<Scalar>::epsilon()) {
        s = sin(angle / 2) / angle;
        c = cos(angle / 2);
    } else {
        s = Scalar{0.5} + angle2 / 48;
        c = 1 - angle / 8;
    }

    // storage order x, y, z, w
    q.coeffs() << s * v, c;
    return q.matrix();
}


template <Mat3 (*Fn)(const Vec3 &)>
void BM_expMapFn(benchmark::State &state) {
    const auto N = 1000;
    const auto norm = std::pow(10.0, state.range(0));
    const auto vs = randomSmallVectors<Vec3>(N, norm);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            const Mat3 result = Fn(vs[i]);

            benchmark::DoNotOptimize(result);
            DEBUG_ASSERT_APPROX(Fn(vs[i]), result);
        }
    }
}


// Powers of 10 to use as the norms of the input rotation vectors
const int lopow = -5;
const int hipow = -1;

BENCHMARK_TEMPLATE(BM_expMapFn, expMapQM)->Range(lopow, hipow);
BENCHMARK_TEMPLATE(BM_expMapFn, expMapQMNoCheck)->Range(lopow, hipow);
BENCHMARK_TEMPLATE(BM_expMapFn, expMapM)->Range(lopow, hipow);
BENCHMARK_TEMPLATE(BM_expMapFn, expMapMNoCheck)->Range(lopow, hipow);

WAVE_BENCHMARK_MAIN()
