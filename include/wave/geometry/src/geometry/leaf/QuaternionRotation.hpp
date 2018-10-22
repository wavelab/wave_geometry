/**
 * @file
 */

#ifndef WAVE_GEOMETRY_QUATERNIONROTATION_HPP
#define WAVE_GEOMETRY_QUATERNIONROTATION_HPP

namespace wave {

/** A rotation on SO(3) stored as a quaternion
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Quaterniond or
 * Eigen::Map<Eigen::Quaterniond>)
 *
 * The alias RotationQd is provided for the typical storage type, Eigen::Quaterniond.
 */
template <typename ImplType>
class QuaternionRotation : public RotationBase<QuaternionRotation<ImplType>>,
                           public LeafStorage<QuaternionRotation<ImplType>, ImplType> {
    static_assert(internal::is_eigen_quaternion<ImplType>::value,
                  "ImplType must be an Eigen quaternion type.");
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using TangentType = RelativeRotation<Eigen::Matrix<Scalar, 3, 1>>;
    using Storage = LeafStorage<QuaternionRotation<ImplType>, ImplType>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    /** Constructs uninitialized rotation */
    QuaternionRotation() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(QuaternionRotation)

    /** Constructs from an Eigen 3x3 matrix */
    template <typename MatrixDerived,
              TICK_REQUIRES(internal::is_eigen_matrix<3, MatrixDerived>{})>
    explicit QuaternionRotation(const Eigen::MatrixBase<MatrixDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Constructs from an Eigen rotation object, including Quaternion and AngleAxis.
     */
    template <typename RDerived>
    explicit QuaternionRotation(const Eigen::RotationBase<RDerived, 3> &r)
        : Storage{typename Storage::init_storage{}, r.derived()} {}
};

namespace internal {

template <typename ImplType>
struct traits<QuaternionRotation<ImplType>>
    : rotation_leaf_traits_base<QuaternionRotation<ImplType>> {
    using PlainType =
      QuaternionRotation<typename Eigen::internal::traits<ImplType>::PlainObject>;
    using typename rotation_leaf_traits_base<QuaternionRotation<ImplType>>::Scalar;

    using ConvertTo = tmp::type_list<MatrixRotation<Eigen::Matrix<Scalar, 3, 3>>>;
};


/** Implements inverse of a quaternion */
template <typename Rhs>
auto evalImpl(expr<Inverse>, const QuaternionRotation<Rhs> &m) {
    // Use Eigen's implementation, assuming valid rotation quaternion
    return makeLeaf<QuaternionRotation>(m.derived().value().conjugate());
}

/** Jacobian of inverse of a quaternion */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<Inverse>,
                  const QuaternionRotation<Val> &q_inv,
                  const QuaternionRotation<Rhs> &)
  -> jacobian_t<QuaternionRotation<Val>, QuaternionRotation<Rhs>> {
    return -q_inv.value().toRotationMatrix();
}

// No log map of quaternion. It will be automatically converted to rotation matrix
// @todo maybe implement

/** Implements composition of quaternions */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Compose>,
              const QuaternionRotation<Lhs> &lhs,
              const QuaternionRotation<Rhs> &rhs) {
    // Use Eigen's implementation
    return plain_eval_t<QuaternionRotation<Lhs>>{lhs.value() * rhs.value()};
}

/** Right jacobian of composition with a quaternion on the lhs */
template <typename Val, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Compose>,
                       const Val &,
                       const QuaternionRotation<Lhs> &lhs,
                       const RotationBase<Rhs> &) -> jacobian_t<Val, Rhs> {
    return lhs.value().toRotationMatrix();
}

/** Rotates a translation by a quaternion */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Rotate>,
              const QuaternionRotation<Lhs> &lhs,
              const Translation<Rhs> &rhs) {
    // Use Eigen's implementation
    return plain_eval_t<Translation<Rhs>>{lhs.value() * rhs.value()};
}

/** Jacobian of rotation by quaternion, wrt to the vector */
template <typename Val, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Rotate>,
                       const Translation<Val> &,
                       const QuaternionRotation<Lhs> &lhs,
                       const Translation<Rhs> &)
  -> jacobian_t<Translation<Val>, QuaternionRotation<Lhs>> {
    // Bloesch equation 68
    return lhs.value().toRotationMatrix();
}

/** Implements "conversion" between QuaternionRotation types
 *
 * While this seems trivial, it is needed for the case the template params are not the
 * same.
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, QuaternionRotation<ToImpl>>,
              const QuaternionRotation<FromImpl> &rhs) {
    return QuaternionRotation<ToImpl>{rhs.derived().value()};
}

/** Converts from quaternion to rotation matrix
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, MatrixRotation<ToImpl>>,
              const QuaternionRotation<FromImpl> &rhs) {
    // Use Eigen's implementation
    return MatrixRotation<ToImpl>{rhs.value().toRotationMatrix()};
}

/** Converts from rotation matrix to quaternion
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, QuaternionRotation<ToImpl>>,
              const MatrixRotation<FromImpl> &rhs) {
    // Use Eigen's implementation
    return QuaternionRotation<ToImpl>{rhs.value()};
}


}  // namespace internal

// Convenience typedefs

using RotationQd = QuaternionRotation<Eigen::Quaterniond>;

template <typename F1, typename F2>
using RotationQFd = Framed<RotationQd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_QUATERNIONROTATION_HPP
