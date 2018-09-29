/**
 * @file
 * @author lkoppel
 * Defines an Eigen expression for the "cross-product" or "hat" matrix of a 3-vector
 */

#ifndef WAVE_GEOMETRY_CROSSMATRIX_HPP
#define WAVE_GEOMETRY_CROSSMATRIX_HPP

#include <Eigen/Geometry>

namespace wave {

/**
 * An Eigen expression for the skew-symmetric "cross-product" or "hat" matrix of a
 * 3-vector. This matrix applies the cross product:
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
template <class VecType>
class CrossMatrix : public Eigen::Matrix<typename VecType::Scalar, 3, 3> {
 public:
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecType, 3);
    using MatrixType = Eigen::Matrix<typename VecType::Scalar, 3, 3>;
    using Nested = typename Eigen::internal::ref_selector<CrossMatrix>::type;
    using VecTypeNested = typename Eigen::internal::ref_selector<VecType>::type;

    using Scalar = typename VecType::Scalar;

    EIGEN_STRONG_INLINE
    explicit CrossMatrix(const VecType &vec)
        : MatrixType{(MatrixType{} << Scalar{0},
                      -vec(2),
                      vec(1),
                      vec(2),
                      Scalar{0},
                      -vec(0),
                      -vec(1),
                      vec(0),
                      Scalar{0})
                       .finished()},
          vec{vec} {}

    using NegativeReturnType = CrossMatrix<typename VecType::NegativeReturnType>;

    EIGEN_DEVICE_FUNC
    inline NegativeReturnType operator-() const {
        return NegativeReturnType{-this->vec};
    }


    VecTypeNested vec;
};


/** Produce a cross matrix*/
template <class VecType>
inline auto crossMatrix(const Eigen::MatrixBase<VecType> &vec) -> CrossMatrix<VecType> {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecType, 3);
    return CrossMatrix<VecType>(vec.derived());
}

template <class VecType>
inline auto crossMatrix(const Eigen::MatrixBase<VecType> &&vec) -> CrossMatrix<VecType> {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VecType, 3);
    return CrossMatrix<VecType>(std::move(vec.derived()));
}

// Forward declaration
template <typename Scalar, int N>
class IdentityMatrix;

}  // namespace wave

namespace Eigen {

/** Left-multiply a fixed-size 3-vector by a cross-matrix
 *
 * Performs a cross product
 */
template <
  typename VecType,
  typename OtherType,
  std::enable_if_t<OtherType::RowsAtCompileTime == 3 && OtherType::ColsAtCompileTime == 1,
                   int> = 0>
EIGEN_DEVICE_FUNC inline auto operator*(const wave::CrossMatrix<VecType> &crossMat,
                                        const Eigen::MatrixBase<OtherType> &rhs) {
    return crossMat.vec.cross(rhs);
}

/** Right-multiply a fixed-size 3-vector by a cross-matrix
 *
 * Performs a cross product
 */
template <
  typename VecType,
  typename OtherType,
  std::enable_if_t<OtherType::ColsAtCompileTime == 3 && OtherType::RowsAtCompileTime == 1,
                   int> = 0>
EIGEN_DEVICE_FUNC inline auto operator*(const Eigen::MatrixBase<OtherType> &lhs,
                                        const wave::CrossMatrix<VecType> &crossMat) {
    return lhs.cross(crossMat.vec);
}

/**
 * Left-multiply a 3*N matrix by a cross-matrix
 *
 * This operation is equivalent to taking the cross product of each row in the l.h.s.
 * matrix with the negative original vector
 *
 * Only enabled for static matrices
 */
template <
  typename VecType,
  typename OtherType,
  std::enable_if_t<OtherType::RowsAtCompileTime == 3 && OtherType::ColsAtCompileTime != 1,
                   int> = 0>
EIGEN_DEVICE_FUNC inline auto operator*(const wave::CrossMatrix<VecType> &crossMat,
                                        const Eigen::MatrixBase<OtherType> &rhs) {
    return rhs.colwise().cross(-crossMat.vec);
}

/**
 * Left-multiply a 3*N matrix by a cross-matrix of a negative
 *
 * This operation is equivalent to taking the cross product of each row in the l.h.s.
 * matrix with the original vector
 *
 * This special case cuts down one instruction(?), but more importantly simplifies the
 * expression graph output.
 */
template <
  typename VecType,
  typename OtherType,
  std::enable_if_t<OtherType::RowsAtCompileTime == 3 && OtherType::ColsAtCompileTime != 1,
                   int> = 0>
EIGEN_DEVICE_FUNC inline auto operator*(
  const wave::CrossMatrix<Eigen::CwiseUnaryOp<
    Eigen::internal::scalar_opposite_op<typename internal::traits<VecType>::Scalar>,
    VecType>> &crossMat,
  const Eigen::MatrixBase<OtherType> &rhs) {
    return rhs.derived().colwise().cross(crossMat.vec.nestedExpression());
}


/**
 * Right-multiply an N*3 matrix by a cross-matrix
 *
 * This operation is equivalent to taking the cross product of the original vector with
 * each column in the r.h.s. matrix, and negating it since a x b = -(b x a).
 *
 * Only enabled for static matrices
 */
template <typename OtherType,
          typename VecType,
          std::enable_if_t<OtherType::ColsAtCompileTime == 3, int> = 0>
EIGEN_DEVICE_FUNC inline auto operator*(const Eigen::MatrixBase<OtherType> &lhs,
                                        const wave::CrossMatrix<VecType> &crossMat) {
    return lhs.rowwise().cross(crossMat.vec);
}

/**
 * Multiply two cross-matrices
 * (needed to disambiguate)
 */
template <typename Lhs, typename Rhs>
EIGEN_DEVICE_FUNC inline auto operator*(const wave::CrossMatrix<Lhs> &lhs,
                                        const wave::CrossMatrix<Rhs> &rhs) {
    return lhs.rowwise().cross(rhs.vec);
}

/**
 * Multiply an Identity expression by a CrossMatrix expression
 * (Provided to break tie between the other specializations)
 */
// @todo generalize - this won't scale well if we add more expressions with operator*
// @todo Check scalar mixing?
template <typename VecType, typename Scalar>
EIGEN_DEVICE_FUNC inline const wave::CrossMatrix<VecType> &operator*(
  const wave::IdentityMatrix<Scalar, 3> &, const wave::CrossMatrix<VecType> &cross) {
    return cross;
}

/**
 * Multiply an CrossMatrix expression by an Identity expression
 * (Provided to break tie between the other specializations)
 */
// @todo generalize - this won't scale well if we add more expressions with operator*
// @todo Check scalar mixing?
template <typename VecType, typename Scalar>
EIGEN_DEVICE_FUNC inline const wave::CrossMatrix<VecType> &operator*(
  const wave::CrossMatrix<VecType> &cross, const wave::IdentityMatrix<Scalar, 3> &) {
    return cross;
}

namespace internal {

// Static attributes of our CrossMatrix expression
// See https://eigen.tuxfamily.org/dox/TopicNewExpressionType.html
template <class VecType>
struct traits<::wave::CrossMatrix<VecType>> {
    using StorageKind = Eigen::Dense;
    using XprKind = Eigen::MatrixXpr;
    using StorageIndex = typename VecType::StorageIndex;
    using Scalar = typename VecType::Scalar;
    enum {
        Flags = Eigen::AutoAlign,
        RowsAtCompileTime = 3,
        ColsAtCompileTime = 3,
        MaxRowsAtCompileTime = 3,
        MaxColsAtCompileTime = 3
    };
};


}  // namespace internal
}  // namespace Eigen

#endif  // WAVE_GEOMETRY_CROSSMATRIX_HPP
