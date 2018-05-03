/**
 * @file
 * Test of rotation-chain example using Ceres autodiff, for comparison
 */
#include <benchmark/benchmark.h>
#include <ceres/ceres.h>
#include <Eigen/Core>

#include "../bechmark_helpers.hpp"

template <typename Scalar>
using Mat3 = Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>;

template <typename Scalar>
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar>
using Mat9 = Eigen::Matrix<Scalar, 9, 9, Eigen::RowMajor>;

template <typename Scalar>
using InVec3 = Eigen::Map<const Vec3<Scalar>>;

template <typename Scalar>
using OutVec3 = Eigen::Map<Vec3<Scalar>>;

template <typename Scalar>
using InMat3 = Eigen::Map<const Mat3<Scalar>>;

template <typename Scalar>
using OutMat3 = Eigen::Map<Mat3<Scalar>>;

struct ChainFunctor1 {
    template <typename T>
    bool operator()(const T *const x0, const T *const x1, T *e) const {
        InVec3<T> v1{x0};
        InMat3<T> R1{x1};
        OutVec3<T> v2{e};

        v2 = R1 * v1;
        return true;
    }
};

struct ChainFunctor2 {
    template <typename T>
    bool operator()(const T *const x0, const T *const x1, const T *const x2, T *e) const {
        InVec3<T> v1{x0};
        InMat3<T> R1{x1};
        InMat3<T> R2{x2};
        OutVec3<T> v2{e};

        v2 = R1 * (R2 * v1);
        return true;
    }
};

struct ChainFunctor3 {
    template <typename T>
    bool operator()(const T *const x0,
                    const T *const x1,
                    const T *const x2,
                    const T *const x3,
                    T *e) const {
        InVec3<T> v1{x0};
        InMat3<T> R1{x1};
        InMat3<T> R2{x2};
        InMat3<T> R3{x3};
        OutVec3<T> v2{e};

        v2 = R1 * (R2 * (R3 * v1));
        return true;
    }
};

struct ChainFunctor4 {
    template <typename T>
    bool operator()(const T *const x0,
                    const T *const x1,
                    const T *const x2,
                    const T *const x3,
                    const T *const x4,
                    T *e) const {
        InVec3<T> v1{x0};
        InMat3<T> R1{x1};
        InMat3<T> R2{x2};
        InMat3<T> R3{x3};
        InMat3<T> R4{x4};
        OutVec3<T> v2{e};

        v2 = R1 * (R2 * (R3 * (R4 * v1)));
        return true;
    }
};

struct ChainFunctor5 {
    template <typename T>
    bool operator()(const T *const x0,
                    const T *const x1,
                    const T *const x2,
                    const T *const x3,
                    const T *const x4,
                    const T *const x5,
                    T *e) const {
        InVec3<T> v1{x0};
        InMat3<T> R1{x1};
        InMat3<T> R2{x2};
        InMat3<T> R3{x3};
        InMat3<T> R4{x4};
        InMat3<T> R5{x5};
        OutVec3<T> v2{e};

        v2 = R1 * (R2 * (R3 * (R4 * (R5 * v1))));
        return true;
    }
};

struct ChainFunctor6 {
    template <typename T>
    bool operator()(const T *const x0,
                    const T *const x1,
                    const T *const x2,
                    const T *const x3,
                    const T *const x4,
                    const T *const x5,
                    const T *const x6,
                    T *e) const {
        InVec3<T> v1{x0};
        InMat3<T> R1{x1};
        InMat3<T> R2{x2};
        InMat3<T> R3{x3};
        InMat3<T> R4{x4};
        InMat3<T> R5{x5};
        InMat3<T> R6{x6};
        OutVec3<T> v2{e};

        v2 = R1 * (R2 * (R3 * (R4 * (R5 * (R6 * v1)))));
        return true;
    }
};


void BM_ceres_Chain1(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);

    auto v1_vec = randomMatrices<Eigen::Vector3d>(N);
    auto R1_vec = randomMatrices<wave::RotationMd>(N);

    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor1, 3, 3, 9>{new ChainFunctor1()}};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Mat3<double> res;
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J0;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J1;
            double const *const inputs[] = {v1_vec[i].data(), R1_vec[i].value().data()};
            double *jacobians[] = {J0.data(), J1.data()};
            cost_function->Evaluate(inputs, res.data(), jacobians);

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
        }
    }
}

void BM_ceres_Chain2(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);

    auto v1_vec = randomMatrices<Eigen::Vector3d>(N);
    auto R1_vec = randomMatrices<wave::RotationMd>(N);
    auto R2_vec = randomMatrices<wave::RotationMd>(N);
    auto R3_vec = randomMatrices<wave::RotationMd>(N);

    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor2, 3, 3, 9, 9>{new ChainFunctor2()}};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Mat3<double> res;
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J0;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J1;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J2;

            double const *const inputs[] = {
              v1_vec[i].data(), R1_vec[i].value().data(), R2_vec[i].value().data()};
            double *jacobians[] = {J0.data(), J1.data(), J2.data()};
            cost_function->Evaluate(inputs, res.data(), jacobians);

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
        }
    }
}

void BM_ceres_Chain3(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);

    auto v1_vec = randomMatrices<Eigen::Vector3d>(N);
    auto R1_vec = randomMatrices<wave::RotationMd>(N);
    auto R2_vec = randomMatrices<wave::RotationMd>(N);
    auto R3_vec = randomMatrices<wave::RotationMd>(N);

    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor3, 3, 3, 9, 9, 9>{new ChainFunctor3()}};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Mat3<double> res;
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J0;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J1;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J2;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J3;

            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().data(),
                                            R2_vec[i].value().data(),
                                            R3_vec[i].value().data()};
            double *jacobians[] = {J0.data(), J1.data(), J2.data(), J3.data()};
            cost_function->Evaluate(inputs, res.data(), jacobians);

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J3);
        }
    }
}

void BM_ceres_Chain4(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);

    auto v1_vec = randomMatrices<Eigen::Vector3d>(N);
    auto R1_vec = randomMatrices<wave::RotationMd>(N);
    auto R2_vec = randomMatrices<wave::RotationMd>(N);
    auto R3_vec = randomMatrices<wave::RotationMd>(N);
    auto R4_vec = randomMatrices<wave::RotationMd>(N);

    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor4, 3, 3, 9, 9, 9, 9>{
        new ChainFunctor4()}};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Mat3<double> res;
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J0;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J1, J2, J3, J4;

            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().data(),
                                            R2_vec[i].value().data(),
                                            R3_vec[i].value().data(),
                                            R4_vec[i].value().data()};
            double *jacobians[] = {J0.data(), J1.data(), J2.data(), J3.data(), J4.data()};
            cost_function->Evaluate(inputs, res.data(), jacobians);

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J3);
            benchmark::DoNotOptimize(J4);
        }
    }
}

void BM_ceres_Chain5(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);

    auto v1_vec = randomMatrices<Eigen::Vector3d>(N);
    auto R1_vec = randomMatrices<wave::RotationMd>(N);
    auto R2_vec = randomMatrices<wave::RotationMd>(N);
    auto R3_vec = randomMatrices<wave::RotationMd>(N);
    auto R4_vec = randomMatrices<wave::RotationMd>(N);
    auto R5_vec = randomMatrices<wave::RotationMd>(N);

    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor5, 3, 3, 9, 9, 9, 9, 9>{
        new ChainFunctor5()}};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Mat3<double> res;
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J0;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J1, J2, J3, J4, J5;

            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().data(),
                                            R2_vec[i].value().data(),
                                            R3_vec[i].value().data(),
                                            R4_vec[i].value().data(),
                                            R5_vec[i].value().data()};
            double *jacobians[] = {
              J0.data(), J1.data(), J2.data(), J3.data(), J4.data(), J5.data()};
            cost_function->Evaluate(inputs, res.data(), jacobians);

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J3);
            benchmark::DoNotOptimize(J4);
            benchmark::DoNotOptimize(J5);
        }
    }
}

void BM_ceres_Chain6(benchmark::State &state) {
    const auto N = state.range(0);
    state.SetComplexityN(N);

    auto v1_vec = randomMatrices<Eigen::Vector3d>(N);
    auto R1_vec = randomMatrices<wave::RotationMd>(N);
    auto R2_vec = randomMatrices<wave::RotationMd>(N);
    auto R3_vec = randomMatrices<wave::RotationMd>(N);
    auto R4_vec = randomMatrices<wave::RotationMd>(N);
    auto R5_vec = randomMatrices<wave::RotationMd>(N);
    auto R6_vec = randomMatrices<wave::RotationMd>(N);

    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor6, 3, 3, 9, 9, 9, 9, 9, 9>{
        new ChainFunctor6()}};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            Mat3<double> res;
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor> J0;
            Eigen::Matrix<double, 3, 9, Eigen::RowMajor> J1, J2, J3, J4, J5, J6;

            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().data(),
                                            R2_vec[i].value().data(),
                                            R3_vec[i].value().data(),
                                            R4_vec[i].value().data(),
                                            R5_vec[i].value().data(),
                                            R6_vec[i].value().data()};
            double *jacobians[] = {J0.data(),
                                   J1.data(),
                                   J2.data(),
                                   J3.data(),
                                   J4.data(),
                                   J5.data(),
                                   J6.data()};
            cost_function->Evaluate(inputs, res.data(), jacobians);

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J3);
            benchmark::DoNotOptimize(J4);
            benchmark::DoNotOptimize(J5);
            benchmark::DoNotOptimize(J6);
        }
    }
}


BENCHMARK(BM_ceres_Chain1)->Arg(1000);
BENCHMARK(BM_ceres_Chain2)->Arg(1000);
BENCHMARK(BM_ceres_Chain3)->Arg(1000);
BENCHMARK(BM_ceres_Chain4)->Arg(1000);
BENCHMARK(BM_ceres_Chain5)->Arg(1000);
BENCHMARK(BM_ceres_Chain6)->Arg(1000);

WAVE_BENCHMARK_MAIN()
