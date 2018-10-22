/**
 * @file
 */

#ifndef WAVE_GEOMETRY_MATRIXROTATION_HPP
#define WAVE_GEOMETRY_MATRIXROTATION_HPP

namespace wave {

/** A rotation on SO(3) stored as a rotation matrix
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Matrix3d or
 * Eigen::Map<Eigen::Matrix3d>)
 *
 * The alias RotationMd is provided for the typical storage type, Eigen::Matrix3d.
 */
template <typename ImplType>
class MatrixRotation : public RotationBase<MatrixRotation<ImplType>>,
                       public LeafStorage<MatrixRotation<ImplType>, ImplType> {
    static_assert(internal::is_eigen_matrix<3, ImplType>::value,
                  "ImplType must be an Eigen 3*3 matrix type.");
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using TangentType = RelativeRotation<Eigen::Matrix<Scalar, 3, 1>>;
    using Storage = LeafStorage<MatrixRotation<ImplType>, ImplType>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    /** Constructs uninitialized rotation */
    MatrixRotation() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(MatrixRotation)

    /** Constructs from an Eigen 3x3 matrix */
    template <typename MatrixDerived,
              TICK_REQUIRES(internal::is_eigen_matrix<3, MatrixDerived>{})>
    explicit MatrixRotation(const Eigen::MatrixBase<MatrixDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Constructs from an Eigen rotation object, including Quaternion and AngleAxis.
     */
    template <typename RDerived>
    explicit MatrixRotation(const Eigen::RotationBase<RDerived, 3> &r)
        : Storage{typename Storage::init_storage{}, r.derived().toRotationMatrix()} {}
};

namespace internal {

template <typename ImplType>
struct traits<MatrixRotation<ImplType>>
    : rotation_leaf_traits_base<MatrixRotation<ImplType>> {
    using PlainType = MatrixRotation<typename ImplType::PlainObject>;
};

/** Implements inverse of a rotation matrix */
template <typename Rhs>
auto evalImpl(expr<Inverse>, const MatrixRotation<Rhs> &m) {
    return makeLeaf<MatrixRotation>(m.derived().value().transpose());
}

/** Jacobian of inverse of a rotation matrix */
template <typename Val, typename Rhs>
decltype(auto) jacobianImpl(expr<Inverse>,
                            const MatrixRotation<Val> &inv,
                            const MatrixRotation<Rhs> &) {
    return -inv.value();
}

/** Implements log map of rotation matrix */
template <typename ImplType>
auto evalImpl(expr<LogMap>, const MatrixRotation<ImplType> &rhs) ->
  typename traits<MatrixRotation<ImplType>>::TangentType {
    using Scalar = scalar_t<MatrixRotation<ImplType>>;

    // From http://ethaneade.com/lie.pdf
    using std::acos;
    using std::sin;
    const auto &m = rhs.value();
    const auto angle = acos((m.trace() - Scalar{1}) / Scalar{2});

    if (angle * angle > Eigen::NumTraits<Scalar>::epsilon()) {
        return uncrossMatrix(angle / (Scalar{2} * sin(angle)) * (m - m.transpose()));
    } else {
        // Very small angle
        return uncrossMatrix(Scalar{0.5} * (m - m.transpose()));
    }
}

/** Implements composition of rotation matrices */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Compose>,
              const MatrixRotation<Lhs> &lhs,
              const MatrixRotation<Rhs> &rhs) {
    return plain_eval_t<MatrixRotation<Lhs>>{lhs.value() * rhs.value()};
}

/** Right jacobian of composition with a rotation matrix on the lhs
 *
 * Since we already have a matrix, we can return a reference. */
template <typename Val, typename Lhs, typename Rhs>
decltype(auto) rightJacobianImpl(expr<Compose>,
                                 const Val &,
                                 const MatrixRotation<Lhs> &lhs,
                                 const RotationBase<Rhs> &) {
    return lhs.value();
}

/** Rotates a translation by a rotation matrix */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Rotate>, const MatrixRotation<Lhs> &lhs, const Translation<Rhs> &rhs) {
    return plain_eval_t<Translation<Rhs>>{lhs.value() * rhs.value()};
}

/** Jacobian of rotation wrt to the vector
 *
 * Since we already have a matrix, we can return a reference. */
template <typename Val, typename Lhs, typename Rhs>
decltype(auto) rightJacobianImpl(expr<Rotate>,
                                 const Translation<Val> &,
                                 const MatrixRotation<Lhs> &lhs,
                                 const Translation<Rhs> &) {
    // Bloesch equation 68
    return lhs.value();
}


/** Implements "conversion" between MatrixRotation types
 *
 * While this seems trivial, it is needed for the case the template params are not the
 * same.
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, MatrixRotation<ToImpl>>,
              const MatrixRotation<FromImpl> &rhs) {
    return MatrixRotation<ToImpl>{rhs.derived().value()};
}

}  // namespace internal

// Convenience typedefs

using RotationMd = MatrixRotation<Eigen::Matrix3d>;

template <typename F1, typename F2>
using RotationMFd = Framed<RotationMd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_MATRIXROTATION_HPP
