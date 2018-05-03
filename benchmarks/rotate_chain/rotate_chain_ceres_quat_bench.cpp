/**
 * @file
 * Test of rotation-chain example using Ceres autodiff, for comparison
 */
#include <benchmark/benchmark.h>
#include <ceres/ceres.h>
#include <Eigen/Core>

#include "../bechmark_helpers.hpp"

template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

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
using InQuat = Eigen::Map<const Eigen::Quaternion<Scalar>>;

template <typename Scalar>
using OutQuat = Eigen::Map<Eigen::Quaternion<Scalar>>;

template <typename Scalar>
using InMat3 = Eigen::Map<const Mat3<Scalar>>;

template <typename Scalar>
using OutMat3 = Eigen::Map<Mat3<Scalar>>;

using InMat34d = Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>;
using InMat43d = Eigen::Map<const Eigen::Matrix<double, 4, 3, Eigen::RowMajor>>;

struct ChainFunctor1 {
    template <typename T>
    bool operator()(const T *const x0, const T *const x1, T *e) const {
        InVec3<T> v1{x0};
        InQuat<T> q1{x1};
        OutVec3<T> v2{e};

        v2 = q1 * v1;
        return true;
    }
};

struct ChainFunctor2 {
    template <typename T>
    bool operator()(const T *const x0, const T *const x1, const T *const x2, T *e) const {
        InVec3<T> v1{x0};
        InQuat<T> q1{x1}, q2{x2};
        OutVec3<T> v2{e};

        v2 = q1 * q2 * v1;
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
        InQuat<T> q1{x1}, q2{x2}, q3{x3};
        OutVec3<T> v2{e};

        v2 = q1 * q2 * q3 * v1;
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
        InQuat<T> q1{x1}, q2{x2}, q3{x3}, q4{x4};
        OutVec3<T> v2{e};

        v2 = q1 * q2 * q3 * q4 * v1;
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
        InQuat<T> q1{x1}, q2{x2}, q3{x3}, q4{x4}, q5{x5};
        OutVec3<T> v2{e};

        v2 = q1 * q2 * q3 * q4 * q5 * v1;
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
        InQuat<T> q1{x1}, q2{x2}, q3{x3}, q4{x4}, q5{x5}, q6{x6};
        OutVec3<T> v2{e};

        v2 = q1 * q2 * q3 * q4 * q5 * q6 * v1;
        return true;
    }
};

struct ChainFunctor7 {
    template <typename T>
    bool operator()(const T *const x0,
                    const T *const x1,
                    const T *const x2,
                    const T *const x3,
                    const T *const x4,
                    const T *const x5,
                    const T *const x6,
                    const T *const x7,
                    T *e) const {
        InVec3<T> v1{x0};
        InQuat<T> q1{x1}, q2{x2}, q3{x3}, q4{x4}, q5{x5}, q6{x6}, q7{x7};
        OutVec3<T> v2{e};

        v2 = q1 * q2 * q3 * q4 * q5 * q6 * q7 * v1;
        return true;
    }
};

class RotateChain : public benchmark::Fixture {
 protected:
    const int N = 1000;
    // Use wave::RotationM just for the Random() method, which produces valid SO(3)
    using waveRot = wave::RotationQd;
    const EigenVector<waveRot> R1_vec = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R2_vec = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R3_vec = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R4_vec = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R5_vec = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R6_vec = randomMatrices<waveRot>(N);
    const EigenVector<waveRot> R7_vec = randomMatrices<waveRot>(N);
    const EigenVector<Eigen::Vector3d> v1_vec = randomMatrices<Eigen::Vector3d>(N);
};

BENCHMARK_F(RotateChain, ceres1)(benchmark::State &state) {
    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor1, 3, 3, 4>{new ChainFunctor1()}};

    ceres::EigenQuaternionParameterization qp{};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            double res[3];
            double J0[3 * 3];
            double J1_global[3 * 4];
            Mat3<double> J1, J2;


            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().coeffs().data()};
            double *jacobians[] = {J0, J1_global};

            // Compute result and global (3*4) jacobians
            cost_function->Evaluate(inputs, res, jacobians);

            // Compute local (3*3) jacobians
            double J_global_local[4 * 3];
            qp.ComputeJacobian(R1_vec[i].value().coeffs().data(), J_global_local);
            J1 = InMat34d{J1_global} * InMat43d{J_global_local};

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
        }
    }
}


BENCHMARK_F(RotateChain, ceres2)(benchmark::State &state) {
    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor2, 3, 3, 4, 4>{new ChainFunctor2()}};

    ceres::EigenQuaternionParameterization qp{};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            double res[3];
            double J0[3 * 3];
            double J1_global[3 * 4], J2_global[3 * 4];
            Mat3<double> J1, J2;


            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().coeffs().data(),
                                            R2_vec[i].value().coeffs().data()};
            double *jacobians[] = {J0, J1_global, J2_global};

            // Compute result and global (3*4) jacobians
            cost_function->Evaluate(inputs, res, jacobians);

            // Compute local (3*3) jacobians
            double J_global_local[4 * 3];
            qp.ComputeJacobian(R1_vec[i].value().coeffs().data(), J_global_local);
            J1 = InMat34d{J1_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R2_vec[i].value().coeffs().data(), J_global_local);
            J2 = InMat34d{J2_global} * InMat43d{J_global_local};

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
        }
    }
}

BENCHMARK_F(RotateChain, ceres3)(benchmark::State &state) {
    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor3, 3, 3, 4, 4, 4>{new ChainFunctor3()}};

    ceres::EigenQuaternionParameterization qp{};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            double res[3];
            double J0[3 * 3];
            double J1_global[3 * 4], J2_global[3 * 4], J3_global[3 * 4];
            Mat3<double> J1, J2, J3;


            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().coeffs().data(),
                                            R2_vec[i].value().coeffs().data(),
                                            R3_vec[i].value().coeffs().data()};
            double *jacobians[] = {J0, J1_global, J2_global, J3_global};

            // Compute result and global (3*4) jacobians
            cost_function->Evaluate(inputs, res, jacobians);

            // Compute local (3*3) jacobians
            double J_global_local[4 * 3];
            qp.ComputeJacobian(R1_vec[i].value().coeffs().data(), J_global_local);
            J1 = InMat34d{J1_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R2_vec[i].value().coeffs().data(), J_global_local);
            J2 = InMat34d{J2_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R3_vec[i].value().coeffs().data(), J_global_local);
            J3 = InMat34d{J3_global} * InMat43d{J_global_local};

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J3);
        }
    }
}

BENCHMARK_F(RotateChain, ceres4)(benchmark::State &state) {
    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor4, 3, 3, 4, 4, 4, 4>{
        new ChainFunctor4()}};

    ceres::EigenQuaternionParameterization qp{};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            double res[3];
            double J0[3 * 3];
            double J1_global[3 * 4], J2_global[3 * 4], J3_global[3 * 4], J4_global[3 * 4];
            Mat3<double> J1, J2, J3, J4;


            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().coeffs().data(),
                                            R2_vec[i].value().coeffs().data(),
                                            R3_vec[i].value().coeffs().data(),
                                            R4_vec[i].value().coeffs().data()};
            double *jacobians[] = {J0, J1_global, J2_global, J3_global, J4_global};

            // Compute result and global (3*4) jacobians
            cost_function->Evaluate(inputs, res, jacobians);

            // Compute local (3*3) jacobians
            double J_global_local[4 * 3];
            qp.ComputeJacobian(R1_vec[i].value().coeffs().data(), J_global_local);
            J1 = InMat34d{J1_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R2_vec[i].value().coeffs().data(), J_global_local);
            J2 = InMat34d{J2_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R3_vec[i].value().coeffs().data(), J_global_local);
            J3 = InMat34d{J3_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R4_vec[i].value().coeffs().data(), J_global_local);
            J4 = InMat34d{J4_global} * InMat43d{J_global_local};

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J3);
            benchmark::DoNotOptimize(J4);
        }
    }
}

BENCHMARK_F(RotateChain, ceres5)(benchmark::State &state) {
    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor5, 3, 3, 4, 4, 4, 4, 4>{
        new ChainFunctor5()}};

    ceres::EigenQuaternionParameterization qp{};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            double res[3];
            double J0[3 * 3];
            double J1_global[3 * 4], J2_global[3 * 4], J3_global[3 * 4], J4_global[3 * 4],
              J5_global[3 * 4];
            Mat3<double> J1, J2, J3, J4, J5;


            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().coeffs().data(),
                                            R2_vec[i].value().coeffs().data(),
                                            R3_vec[i].value().coeffs().data(),
                                            R4_vec[i].value().coeffs().data(),
                                            R5_vec[i].value().coeffs().data()};
            double *jacobians[] = {
              J0, J1_global, J2_global, J3_global, J4_global, J5_global};

            // Compute result and global (3*4) jacobians
            cost_function->Evaluate(inputs, res, jacobians);

            // Compute local (3*3) jacobians
            double J_global_local[4 * 3];
            qp.ComputeJacobian(R1_vec[i].value().coeffs().data(), J_global_local);
            J1 = InMat34d{J1_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R2_vec[i].value().coeffs().data(), J_global_local);
            J2 = InMat34d{J2_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R3_vec[i].value().coeffs().data(), J_global_local);
            J3 = InMat34d{J3_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R4_vec[i].value().coeffs().data(), J_global_local);
            J4 = InMat34d{J4_global} * InMat43d{J_global_local};
            qp.ComputeJacobian(R5_vec[i].value().coeffs().data(), J_global_local);
            J5 = InMat34d{J5_global} * InMat43d{J_global_local};

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

BENCHMARK_F(RotateChain, ceres6)(benchmark::State &state) {
    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor6, 3, 3, 4, 4, 4, 4, 4, 4>{
        new ChainFunctor6()}};

    ceres::EigenQuaternionParameterization qp{};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            double res[3];
            double J0[3 * 3];
            double J_global[6][3 * 4];
            EigenVector<Mat3<double>> J_local(6);


            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().coeffs().data(),
                                            R2_vec[i].value().coeffs().data(),
                                            R3_vec[i].value().coeffs().data(),
                                            R4_vec[i].value().coeffs().data(),
                                            R5_vec[i].value().coeffs().data(),
                                            R6_vec[i].value().coeffs().data()};
            double *jacobians[] = {J0,
                                   J_global[0],
                                   J_global[1],
                                   J_global[2],
                                   J_global[3],
                                   J_global[4],
                                   J_global[5]};

            // Compute result and global (3*4) jacobians
            cost_function->Evaluate(inputs, res, jacobians);

            // Compute local (3*3) jacobians
            double J_global_local[4 * 3];
            for (int i = 0; i < 6; ++i) {
                qp.ComputeJacobian(R1_vec[i].value().coeffs().data(), J_global_local);
                J_local[i] = InMat34d{J_global[i]} * InMat43d{J_global_local};
            }

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J_local);
        }
    }
}

BENCHMARK_F(RotateChain, ceres7)(benchmark::State &state) {
    std::unique_ptr<ceres::CostFunction> cost_function{
      new ceres::AutoDiffCostFunction<ChainFunctor7, 3, 3, 4, 4, 4, 4, 4, 4, 4>{
        new ChainFunctor7()}};

    ceres::EigenQuaternionParameterization qp{};

    for (auto _ : state) {
        for (auto i = N; i-- > 0;) {
            double res[3];
            double J0[3 * 3];
            double J_global[7][3 * 4];
            EigenVector<Mat3<double>> J_local(7);


            double const *const inputs[] = {v1_vec[i].data(),
                                            R1_vec[i].value().coeffs().data(),
                                            R2_vec[i].value().coeffs().data(),
                                            R3_vec[i].value().coeffs().data(),
                                            R4_vec[i].value().coeffs().data(),
                                            R5_vec[i].value().coeffs().data(),
                                            R6_vec[i].value().coeffs().data(),
                                            R7_vec[i].value().coeffs().data()};
            double *jacobians[] = {J0,
                                   J_global[0],
                                   J_global[1],
                                   J_global[2],
                                   J_global[3],
                                   J_global[4],
                                   J_global[5],
                                   J_global[6]};

            // Compute result and global (3*4) jacobians
            cost_function->Evaluate(inputs, res, jacobians);

            // Compute local (3*3) jacobians
            double J_global_local[4 * 3];
            for (int i = 0; i < 7; ++i) {
                qp.ComputeJacobian(R1_vec[i].value().coeffs().data(), J_global_local);
                J_local[i] = InMat34d{J_global[i]} * InMat43d{J_global_local};
            }

            benchmark::DoNotOptimize(res);
            benchmark::DoNotOptimize(J0);
            benchmark::DoNotOptimize(J_local);
        }
    }
}

WAVE_BENCHMARK_MAIN()
