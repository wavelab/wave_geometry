#include <benchmark/benchmark.h>

#include "wave/geometry/geometry.hpp"
#include "wave/geometry/debug.hpp"
#include "../bechmark_helpers.hpp"

using namespace wave;

template <int I>
struct FrameN;

template <typename T>
using EigenVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <class I, class J>
using RMFd = wave::RotationMFd<I, J>;

template <class I, class J>
using RMFdVector = EigenVector<RMFd<I, J>>;

template <class I, class J>
RMFd<I, J> randomRMFd() {
    return RMFd<I, J>::Random();
};

struct FrameW;
struct FrameI;
struct FrameJ;

class Imu : public benchmark::Fixture {
 protected:
    const int N = 1000;
    const RMFdVector<FrameI, FrameJ> meas_Rij = randomMatrices<RMFd<FrameI, FrameJ>>(N);
    const RMFdVector<FrameW, FrameI> R_i = randomMatrices<RMFd<FrameW, FrameI>>(N);
    const RMFdVector<FrameW, FrameJ> R_j = randomMatrices<RMFd<FrameW, FrameJ>>(N);
    const EigenVector<RelativeRotationFd<FrameJ, FrameJ, FrameJ>> wg =
      randomMatrices<RelativeRotationFd<FrameJ, FrameJ, FrameJ>>(N);
};

BENCHMARK_F(Imu, waveTyped)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i--;) {
            const auto &expr1 =
              inverse(meas_Rij[i] * exp(wg[i])) * inverse(R_i[i]) * R_j[i];
            const auto &expr = log(expr1);

            auto [r, J1, J2, J_phi_i, J_phi_j] =
              expr.evalWithJacobians(meas_Rij[i], wg[i], R_i[i], R_j[i]);
            benchmark::DoNotOptimize(r);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J_phi_i);
            benchmark::DoNotOptimize(J_phi_j);
        }
    }
}

BENCHMARK_F(Imu, waveReverse)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i--;) {
            const auto &expr1 =
              inverse(meas_Rij[i] * exp(wg[i])) * inverse(R_i[i]) * R_j[i];
            const auto &expr = log(expr1);


            auto [r, J1, J2, J_phi_i, J_phi_j] =
              wave::internal::evaluateWithReverseJacobians(expr);
            benchmark::DoNotOptimize(r);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J_phi_i);
            benchmark::DoNotOptimize(J_phi_j);
        }
    }
}

BENCHMARK_F(Imu, waveUntyped)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i--;) {
            const auto &expr1 =
              inverse(meas_Rij[i] * exp(wg[i])) * inverse(R_i[i]) * R_j[i];
            const auto &expr = log(expr1);

            auto [r, J1, J2, J_phi_i, J_phi_j] = wave::internal::evaluateWithJacobians(
              expr, meas_Rij[i], wg[i], R_i[i], R_j[i]);
            benchmark::DoNotOptimize(r);
            benchmark::DoNotOptimize(J1);
            benchmark::DoNotOptimize(J2);
            benchmark::DoNotOptimize(J_phi_i);
            benchmark::DoNotOptimize(J_phi_j);
        }
    }
}

Eigen::Matrix3d expMap(const Eigen::Vector3d &phi) {
    const auto &q =
      evalImpl(wave::internal::expr<ExpMap>{}, wave::RelativeRotationd{phi});
    return q.value().matrix();
}

Eigen::Vector3d logMap(const Eigen::Matrix3d &C) {
    return evalImpl(wave::internal::expr<LogMap>{}, wave::RotationMd{C}).value();
}

Eigen::Matrix3d jacOfExpMap(const Eigen::Matrix3d val, const Eigen::Vector3d &phi) {
    // Bloesch Equation 80, with adjustment for near-zero case
    const auto pcross = crossMatrix(phi);  // cross-matrix of the so(3) input
    const auto n2 = phi.squaredNorm();     // squared norm of the input
    const auto &C = val;                   // Rotation matrix of the SO(3) output
    if (n2 > Eigen::NumTraits<double>::epsilon()) {
        return ((Eigen::Matrix3d::Identity() - C) * pcross + (phi * phi.transpose())) /
               n2;
    } else {
        return Eigen::Matrix3d::Identity() + 0.5 * pcross;
    }
}

Eigen::Matrix3d jacOfLogMap(const Eigen::Vector3d &phi) {
    // From http://ethaneade.org/exp_diff.pdf
    using std::cos;
    using std::sin;
    const auto theta2 = phi.squaredNorm();
    const auto theta = sqrt(theta2);
    const auto A = sin(theta) / theta;
    const auto B = (1 - cos(theta)) / theta2;

    //        if (theta2 > Eigen::NumTraits<Scalar>::epsilon()) {
    return Eigen::Matrix3d::Identity() - 0.5 * wave::crossMatrix(phi) +
           ((B - 0.5 * A) / (1 - cos(theta))) * wave::crossMatrix(phi) *
             wave::crossMatrix(phi);
    //        } else {
    // @todo small input
    //        }
}


BENCHMARK_F(Imu, hand)(benchmark::State &state) {
    for (auto _ : state) {
        for (auto i = N; i--;) {
            const auto &delta_R_ij = this->meas_Rij[i].value();
            const auto &wg = this->wg[i].value();
            const auto &R_i = this->R_i[i].value();
            const auto &R_j = this->R_j[i].value();

            Eigen::Matrix3d expwg = expMap(wg);
            Eigen::Matrix3d t0 = (delta_R_ij * expwg).transpose();
            Eigen::Matrix3d t1 = t0 * R_i.transpose();
            Eigen::Vector3d r = logMap(t1 * R_j);

            Eigen::Matrix3d Jlogmap = jacOfLogMap(r);
            Eigen::Matrix3d J_R = Jlogmap * (-t0);
            Eigen::Matrix3d J_wg = J_R * delta_R_ij * jacOfExpMap(expwg, wg);
            Eigen::Matrix3d J_phi_j = Jlogmap * t1;
            Eigen::Matrix3d J_phi_i = -J_phi_j;

            benchmark::DoNotOptimize(r);
            benchmark::DoNotOptimize(J_R);
            benchmark::DoNotOptimize(J_wg);
            benchmark::DoNotOptimize(J_phi_i);
            benchmark::DoNotOptimize(J_phi_j);
        }
    }
}


WAVE_BENCHMARK_MAIN()
