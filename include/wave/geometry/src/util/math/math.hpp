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

}  // namespace wave

#endif  // WAVE_GEOMETRY_UTIL_MATH_HPP
