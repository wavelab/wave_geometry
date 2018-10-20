#include <benchmark/benchmark.h>
#include "wave/geometry/src/util/math/CrossMatrix.hpp"
#include "bechmark_helpers.hpp"

namespace {
/** Returns the skew-symmetric "cross-product" or "hat" matrix of a 3-vector
 * This was my original implementation before making the CrossMatrix expression */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> manualCrossMatrix(
  const Eigen::MatrixBase<Derived> &v) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    using S = typename Derived::Scalar;
    Eigen::Matrix<S, 3, 3> m;
    m << S{0}, -v.z(), v.y(),  //
      v.z(), S{0}, -v.x(),     //
      -v.y(), v.x(), S{0};
    return m;
};
}  // namespace

// Functors which call both functions under test, for use with templated benchmarks
struct ManualCross {
    template <typename Derived>
    static auto call(const Eigen::MatrixBase<Derived> &v) {
        return manualCrossMatrix(v);
    }
};

struct WaveCross {
    template <typename Derived>
    static auto call(const Eigen::MatrixBase<Derived> &v) {
        return wave::crossMatrix(v);
    }
};

void BM_ManualCrossMatrix(benchmark::State &state) {
    Eigen::Vector3d vec = Eigen::Vector3d::Random();

    for (auto _ : state) {
        Eigen::Matrix3d result = manualCrossMatrix(vec);
        benchmark::DoNotOptimize(result.data());
    }
}

void BM_ExprCrossMatrix(benchmark::State &state) {
    Eigen::Vector3d vec = Eigen::Vector3d::Random();

    for (auto _ : state) {
        Eigen::Matrix3d result = wave::crossMatrix(vec);
        benchmark::DoNotOptimize(result.data());
    }
}

void BM_ExprEvalCross(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);

    auto a = randomMatrices<Eigen::Vector3d>(N);
    auto b = randomMatrices<Eigen::Vector3d>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Vector3d result = wave::crossMatrix(a[i]) * b[i];
            benchmark::DoNotOptimize(result.data());
        }
    }
}

void BM_ManualEvalCross(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);

    auto a = randomMatrices<Eigen::Vector3d>(N);
    auto b = randomMatrices<Eigen::Vector3d>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Vector3d result = manualCrossMatrix(a[i]) * b[i];
            benchmark::DoNotOptimize(result.data());
        }
    }
}

template <typename Functor,
          typename Vec = Eigen::Vector3d,
          typename Rhs = Eigen::Matrix3d>
void BM_CrossTimesMatrix(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Vec>(N);
    auto b = randomMatrices<Rhs>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Rhs result = Functor::call(a[i]) * b[i];
            benchmark::DoNotOptimize(result);
        }
    }
}

template <typename Functor,
          typename Vec = Eigen::Vector3d,
          typename Lhs = Eigen::Matrix3d>
void BM_MatrixTimesCross(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Vec>(N);
    auto b = randomMatrices<Lhs>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Lhs result = b[i] * Functor::call(a[i]);
            benchmark::DoNotOptimize(result);
        }
    }
}

template <typename Functor>
void BM_CrossTimesCross(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Eigen::Vector3d>(N + 1);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Matrix3d result = Functor::call(a[i]) * Functor::call(a[i + 1]);
            benchmark::DoNotOptimize(result);
        }
    }
}

void BM_HandCodedCross(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Eigen::Vector3d>(N);
    auto b = randomMatrices<Eigen::Vector3d>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Vector3d result = a[i].cross(b[i]);
            benchmark::DoNotOptimize(result);
        }
    }
}

template <typename Functor,
          typename Vec = Eigen::Vector3d,
          typename Rhs = Eigen::Matrix3d>
void BM_NegativeCrossTimesMatrix(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Vec>(N);
    auto b = randomMatrices<Rhs>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Rhs result = (-Functor::call(a[i])) * b[i];
            benchmark::DoNotOptimize(result);
        }
    }
}

template <typename Functor,
          typename Vec = Eigen::Vector3d,
          typename Lhs = Eigen::Matrix3d>
void BM_MatrixTimesCrossNeg(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Vec>(N);
    auto b = randomMatrices<Lhs>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Lhs result = b[i] * Functor::call(-a[i]);
            benchmark::DoNotOptimize(result);
        }
    }
}

template <typename Functor>
void BM_CrossTimesIdentity(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Eigen::Vector3d>(N);
    Eigen::Matrix3d result;

    for (auto _ : state) {
        for (auto i = N; i--;) {
            result = Functor::call(a[i]) * 1;
            benchmark::DoNotOptimize(result);
        }
    }
}


template <typename Functor>
void BM_EvalCross(benchmark::State &state) {
    const auto N = state.range(0);
    auto a = randomMatrices<Eigen::Vector3d>(N);

    for (auto _ : state) {
        for (auto i = N; i--;) {
            Eigen::Matrix3d result = Functor::call(a[i]);
            benchmark::DoNotOptimize(result);
        }
    }
}

// BENCHMARK(BM_ManualCrossMatrix);
// BENCHMARK(BM_ExprCrossMatrix);
// BENCHMARK(BM_ExprEvalCross)->Range(1, 1 << 10)->Complexity(benchmark::oN);
// BENCHMARK(BM_ManualEvalCross)->Range(1, 1 << 10)->Complexity(benchmark::oN);

const int reps = 1000;
BENCHMARK_TEMPLATE(BM_EvalCross, ManualCross)->Arg(reps);
BENCHMARK_TEMPLATE(BM_EvalCross, WaveCross)->Arg(reps);

BENCHMARK_TEMPLATE(BM_CrossTimesIdentity, ManualCross)->Arg(reps);
BENCHMARK_TEMPLATE(BM_CrossTimesIdentity, WaveCross)->Arg(reps);

BENCHMARK_TEMPLATE(BM_CrossTimesMatrix, ManualCross, Eigen::Vector3d, Eigen::Vector3d)
  ->Arg(reps);
BENCHMARK_TEMPLATE(BM_CrossTimesMatrix, WaveCross, Eigen::Vector3d, Eigen::Vector3d)
  ->Arg(reps);

BENCHMARK(BM_HandCodedCross)->Arg(reps);

BENCHMARK_TEMPLATE(BM_CrossTimesMatrix, ManualCross)->Arg(reps);
BENCHMARK_TEMPLATE(BM_CrossTimesMatrix, WaveCross)->Arg(reps);

BENCHMARK_TEMPLATE(BM_NegativeCrossTimesMatrix, ManualCross)->Arg(reps);
BENCHMARK_TEMPLATE(BM_NegativeCrossTimesMatrix, WaveCross)->Arg(reps);

BENCHMARK_TEMPLATE(BM_MatrixTimesCrossNeg, ManualCross)->Arg(reps);
BENCHMARK_TEMPLATE(BM_MatrixTimesCrossNeg, WaveCross)->Arg(reps);

BENCHMARK_TEMPLATE(BM_MatrixTimesCross, ManualCross)->Arg(reps);
BENCHMARK_TEMPLATE(BM_MatrixTimesCross, WaveCross)->Arg(reps);

BENCHMARK_TEMPLATE(BM_CrossTimesCross, ManualCross)->Arg(reps);
BENCHMARK_TEMPLATE(BM_CrossTimesCross, WaveCross)->Arg(reps);

BENCHMARK_TEMPLATE(BM_CrossTimesCross, ManualCross)->Arg(reps);
BENCHMARK_TEMPLATE(BM_CrossTimesCross, WaveCross)->Arg(reps);


BENCHMARK_MAIN();
