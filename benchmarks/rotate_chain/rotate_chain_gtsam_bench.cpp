#include <Eigen/Core>
// Some adaptions for gtsam's bundled Eigen being old
#if !(EIGEN_VERSION_AT_LEAST(3, 3, 0))
#define EIGEN_DEVICE_FUNC
namespace Eigen {
using Index = EIGEN_DEFAULT_DENSE_INDEX_TYPE;
}
#endif

#include <benchmark/benchmark.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/slam/expressions.h>
#include "wave/geometry/src/util/math/math.hpp"


template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;


using gtsam::Expression;
using gtsam::Point3;
using gtsam::Rot3;

void BM_gtsamAll(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);
    // Produce the expression tree
    gtsam::Values values;
    auto p1 = Expression<Point3>{'p', 1};
    auto expr = p1;
    values.insert(gtsam::Symbol{'p', 1}, Point3{Eigen::Vector3d::Random()});

    for (auto i = 0u; i < N; ++i) {
        expr = Expression<Point3>{rotate(Expression<Rot3>{'R', i}, expr)};
        values.insert(gtsam::Symbol{'R', i}, Rot3{wave::randomQuaternion<double>()});
    }
    std::vector<gtsam::Matrix> jacs(N + 1);

    for (auto _ : state) {
        Point3 p2 = expr.value(values, jacs);

        benchmark::DoNotOptimize(p2);
        benchmark::DoNotOptimize(jacs);
    }
}

// This benchmark tests gtsam's valueAndJacobianMap() function, which is normally private
#ifdef LK_CUSTOM_GTSAM_PUBLIC
void BM_gtsamPrivate(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);
    // Produce the expression tree
    gtsam::Values values;
    auto p1 = Expression<Point3>{'p', 1};
    auto expr = p1;
    values.insert(gtsam::Symbol{'p', 1}, Point3{Eigen::Vector3d::Random()});

    for (auto i = 0u; i < N; ++i) {
        expr = Expression<Point3>{rotate(Expression<Rot3>{'R', i}, expr)};
        values.insert(gtsam::Symbol{'R', i}, Rot3{wave::randomQuaternion<double>()});
    }
    std::vector<gtsam::Matrix> jacs(N + 1);

    for (auto _ : state) {
        // Pre-allocate and zero VerticalBlockMatrix
        gtsam::KeyVector keys;
        gtsam::FastVector<int> dims;
        boost::tie(keys, dims) = expr.keysAndDims();
        static const int Dim = gtsam::traits<Point3>::dimension;
        gtsam::VerticalBlockMatrix Ab(dims, Dim);
        Ab.matrix().setZero();
        gtsam::internal::JacobianMap jacobianMap(keys, Ab);

        // Call unsafe version
        Point3 p2 = expr.valueAndJacobianMap(values, jacobianMap);

        benchmark::DoNotOptimize(p2);
        benchmark::DoNotOptimize(values);
        benchmark::DoNotOptimize(jacobianMap);
    }
}
BENCHMARK(BM_gtsamPrivate)->RangeMultiplier(2)->Range(1, 1 << 14)->Complexity();
#endif

BENCHMARK(BM_gtsamAll)->RangeMultiplier(2)->Range(1, 1 << 14)->Complexity();
BENCHMARK_MAIN();
