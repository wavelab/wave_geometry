/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_UTIL_MATH_HPP
#define WAVE_GEOMETRY_UTIL_MATH_HPP

#include <Eigen/Core>
#include <random>

namespace wave {

/** Generate a random real number on the closed interval [a, b] */
template <typename Real>
Real uniformRandom(Real a, Real b) {
    // Use Mersenne Twister pseudo-random number generator, seeded by system random device
    static std::mt19937 rng{std::random_device{}()};

    // The distribution is normally open on one end [a, b). Do this to close it
    const auto closed_b = std::nextafter(b, std::numeric_limits<Real>::max());

    // Pick a real from the distribution
    return std::uniform_real_distribution<Real>{a, closed_b}(rng);
}


/** Generate a random unit quaternion on SO(3)
 *
 * Implements Algorithm 2 from Kuffner, James J. "Effective Sampling and Distance Metrics
 * for 3D Rigid Body Path Planning."
 * https://www.ri.cmu.edu/pub_files/pub4/kuffner_james_2004_1/kuffner_james_2004_1.pdf
 */
template <typename Real>
Eigen::Quaternion<Real> randomQuaternion() {
    const Real s = uniformRandom(Real{0}, Real{1});
    const Real s1 = std::sqrt(1 - s);
    const Real s2 = std::sqrt(s);
    const Real t1 = Real{2 * M_PI} * uniformRandom(Real{0}, Real{1});
    const Real t2 = Real{2 * M_PI} * uniformRandom(Real{0}, Real{1});
    return Eigen::Quaternion<Real>{
      std::cos(t2) * s2, std::sin(t1) * s1, std::cos(t1) * s1, std::sin(t2) * s2};
}

/** Go from a skew-symmetric (cross) matrix to a compact vector
 *
 * Also known as the "vee" operator.
 */
template <typename Derived>
auto uncrossMatrix(const Eigen::MatrixBase<Derived> &skew)
  -> Eigen::Matrix<typename Eigen::internal::traits<Derived>::Scalar, 3, 1> {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
    return Eigen::Matrix<typename Eigen::internal::traits<Derived>::Scalar, 3, 1>{
      skew(2, 1), skew(0, 2), skew(1, 0)};
}

/** Calculates exponential map of a rotation vector into a quaternion
 *
 * When evaluating a rotation matrix, a conversion is needed. However, as shown by
 * expmap_bench.cpp, this method is faster than the Rodrigues formula expmap to matrix,
 * even after the conversion.
 *
 * Based on:  F. S. Grassia, "Practical parameterization of rotations using the
 * exponential map," Journal of graphics tools, 1998.
 */
template <typename Derived>
inline auto quaternionFromExpMap(const Eigen::MatrixBase<Derived> &rotation_vec)
  -> Eigen::Quaternion<typename Derived::Scalar> {
    using Scalar = typename Derived::Scalar;
    using std::cos;
    using std::sin;
    using std::sqrt;
    const auto &v = rotation_vec.derived();
    const Scalar angle2 = v.squaredNorm();
    const Scalar angle = sqrt(angle2);
    Scalar s;
    Scalar c;

    if (angle2 * angle2 > Eigen::NumTraits<Scalar>::epsilon()) {
        const Scalar sa = sin(angle / 2);
        s = sa / angle;
        c = cos(angle / 2);
    } else {
        s = Scalar{0.5} + angle2 / 48;
        c = 1 - angle2 / 8;
    }

    Eigen::Quaternion<Scalar> q{};
    // storage order x, y, z, w
    q.coeffs() << s * v, c;
    return q;
}

/** Calculates "logarithmic map" of a quaternion, obtaining a rotation vector
 */
template <typename Derived>
inline auto rotationVectorFromQuaternion(const Eigen::QuaternionBase<Derived> &quaternion)
  -> Eigen::Matrix<typename Derived::Scalar, 3, 1> {
    using Scalar = typename Derived::Scalar;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using std::abs;
    using std::atan2;
    const auto &q = quaternion.derived();

    const Scalar norm = q.vec().norm();
    if (norm > Eigen::NumTraits<Scalar>::epsilon()) {
        return Vec3{q.vec() / norm * 2 * atan2(norm, abs(q.w()))};
    } else {
        // limit as q.w() -> 1
        return Vec3{2 * q.vec()};
    }
}

/** Calculates logarithmic map of a rotation matrix, obtaining a rotation vector
 */
template <typename Derived>
inline auto rotationVectorFromMatrix(const Eigen::MatrixBase<Derived> &rotation_mat)
  -> Eigen::Matrix<typename Derived::Scalar, 3, 1> {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
    using Scalar = typename Derived::Scalar;
    using std::acos;
    using std::sin;
    const auto &m = rotation_mat.derived();

    // From http://ethaneade.com/lie.pdf
    const auto angle = acos((m.trace() - Scalar{1}) / Scalar{2});

    if (angle * angle > Eigen::NumTraits<Scalar>::epsilon()) {
        return uncrossMatrix(angle / (Scalar{2} * sin(angle)) * (m - m.transpose()));
    } else {
        // Very small angle
        return uncrossMatrix(Scalar{0.5} * (m - m.transpose()));
    }
}


}  // namespace wave

#endif  // WAVE_GEOMETRY_UTIL_MATH_HPP
