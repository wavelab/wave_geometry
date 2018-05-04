#include "wave/geometry/geometry.hpp"
#include "test.hpp"

struct FrameW;
struct FrameI;
struct FrameJ;

using namespace wave;
TEST(Imu, exprJacobians) {
    const auto delta_R_ij = RotationMFd<FrameI, FrameJ>::Random();
    const auto wg = RelativeRotationFd<FrameJ, FrameJ, FrameJ>::Random();
    const auto R_i = RotationMFd<FrameW, FrameI>::Random();
    const auto R_j = RotationMFd<FrameW, FrameJ>::Random();

    const auto &expr1 = inverse(delta_R_ij * exp(wg)) * inverse(R_i) * R_j;
    const auto &expr = log(expr1);

    CHECK_JACOBIANS(true, expr, delta_R_ij, wg, R_i, R_j);
}

Eigen::Matrix3d expMap(const Eigen::Vector3d &phi) {
    return evalImpl(wave::internal::expr<ExpMap>{}, wave::RelativeRotationd{phi}).value();
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


TEST(Imu, compareToHand) {
    const auto delta_R_ij = RotationMFd<FrameI, FrameJ>::Random();
    const auto wg = RelativeRotationFd<FrameJ, FrameJ, FrameJ>::Random();
    const auto R_i = RotationMFd<FrameW, FrameI>::Random();
    const auto R_j = RotationMFd<FrameW, FrameJ>::Random();

    const auto &expr1 = inverse(delta_R_ij * exp(wg)) * inverse(R_i) * R_j;
    const auto &expr = log(expr1);

    wave::RelativeRotationFd<FrameJ, FrameJ, FrameJ> res;
    Eigen::Matrix3d res1, res2, res3, res4;


    std::tie(res, res1, res2, res3, res4) =
      expr.evalWithJacobians(delta_R_ij, wg, R_i, R_j);

    Eigen::Matrix3d expwg = expMap(wg.value());
    Eigen::Matrix3d t0 = (delta_R_ij.value() * expwg).transpose();
    Eigen::Matrix3d t1 = (t0) *R_i.value().transpose();
    Eigen::Vector3d expected = logMap(t1 * R_j.value());

    Eigen::Matrix3d Jlogmap = jacOfLogMap(expected);
    Eigen::Matrix3d J_R = Jlogmap * (-t0);
    Eigen::Matrix3d J_wg = J_R * delta_R_ij.value() * jacOfExpMap(expwg, wg.value());
    Eigen::Matrix3d J_phi_j = Jlogmap * t1;
    Eigen::Matrix3d J_phi_i = -J_phi_j;

    EXPECT_APPROX(expected, res.value());
    EXPECT_APPROX(J_R, res1);
    EXPECT_APPROX(J_wg, res2);
    EXPECT_APPROX(J_phi_i, res3);
    EXPECT_APPROX(J_phi_j, res4);
}
