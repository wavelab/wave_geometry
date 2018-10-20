/**
 * @file
 * @author lkoppel
 * Defines an Eigen expression for a square identity matrix, which can be trivially
 * multiplied.
 */

#ifndef WAVE_GEOMETRY_IDENTITYMATRIX_HPP
#define WAVE_GEOMETRY_IDENTITYMATRIX_HPP

#include <Eigen/Geometry>
#include "wave/geometry/src/util/meta/template_helpers.hpp"

namespace wave {

/**
 * An Eigen expression for a square identity matrix
 *
 * @f[ a \times b = \text{CrossMatrix}(a) b @f]
 *
 * In different works, `CrossMatrix(a)` might be written as @f$ a^{\times} @f$ or
 * @f$ \hat{a} @f$ .
 *
 * @tparam VecType the type of the vector expression
 */
// The implementation of an Eigen expression here was guided by
// https://eigen.tuxfamily.org/dox/TopicNewExpressionType.html
template <typename Scalar, int N>
class IdentityMatrix : public Eigen::Matrix<Scalar, N, N>::IdentityReturnType {
 public:
    using MatrixType = Eigen::Matrix<Scalar, N, N>;
    using Base = typename MatrixType::IdentityReturnType;
    IdentityMatrix() : Base{MatrixType::Identity()} {}
};

}  // namespace wave

namespace Eigen {

namespace internal {

// Static attributes of our Identity expression
// See https://eigen.tuxfamily.org/dox/TopicNewExpressionType.html
template <typename Scalar, int N>
struct traits<::wave::IdentityMatrix<Scalar, N>>
  : Eigen::Matrix<Scalar, N, N>::IdentityReturnType {};

}  // namespace internal

/**
 * Multiply an Identity expression by another matrix on the right
 */
//@todo Consider scalar mixing?
template <typename Scalar, int N, typename OtherDerived>
EIGEN_DEVICE_FUNC inline const OtherDerived &operator*(
  const wave::IdentityMatrix<Scalar, N> &, const Eigen::MatrixBase<OtherDerived> &rhs) {
    static_assert(OtherDerived::RowsAtCompileTime == N ||
                    OtherDerived::RowsAtCompileTime == Eigen::Dynamic,
                  "Invalid matrix product");
    return rhs.derived();
}

/**
 * Multiply an Identity expression by another matrix on the left
 */
//@todo Consider scalar mixing?
template <typename Scalar, int N, typename OtherDerived>
EIGEN_DEVICE_FUNC inline const OtherDerived &operator*(
  const Eigen::MatrixBase<OtherDerived> &lhs, const wave::IdentityMatrix<Scalar, N> &) {
    static_assert(OtherDerived::ColsAtCompileTime == N ||
                    OtherDerived::ColsAtCompileTime == Eigen::Dynamic,
                  "Invalid matrix product");
    return lhs.derived();
}

/**
 * Multiply an Identity expression by another Identity expression
 * (Provided to break tie between the other two specializations)
 */
template <typename Scalar, int N>
EIGEN_DEVICE_FUNC inline const wave::IdentityMatrix<Scalar, N> &operator*(
  const wave::IdentityMatrix<Scalar, N> &i, const wave::IdentityMatrix<Scalar, N> &) {
    return i;
}

}  // namespace Eigen

#endif  // WAVE_GEOMETRY_IDENTITYMATRIX_HPP
