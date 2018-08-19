/**
 * @file
 */

#ifndef WAVE_GEOMETRY_ANGLEAXISROTATION_HPP
#define WAVE_GEOMETRY_ANGLEAXISROTATION_HPP

namespace wave {

/** A rotation on SO(3) stored as an angle and axis
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::AngleAxisd)
 *
 * The alias RotationAd is provided for the typical storage type, Eigen::AngleAxisd.
 */
template <typename ImplType>
class AngleAxisRotation : public RotationBase<AngleAxisRotation<ImplType>>,
                          public LeafExpression<ImplType, AngleAxisRotation<ImplType>> {
    static_assert(internal::is_eigen_angleaxis<ImplType>::value,
                  "ImplType must be an Eigen AngleAxis type.");
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using TangentType = RelativeRotation<Eigen::Matrix<Scalar, 3, 1>>;
    using Storage = LeafExpression<ImplType, AngleAxisRotation<ImplType>>;

 public:
    // Inherit constructors from LeafExpression
    using Storage::Storage;
    using Storage::operator=;

    /** Constructs uninitialized rotation */
    AngleAxisRotation() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(AngleAxisRotation)


    /** Constructs from an Eigen 3x3 matrix */
    template <typename MatrixDerived,
              TICK_REQUIRES(internal::is_eigen_matrix<3, MatrixDerived>{})>
    explicit AngleAxisRotation(const Eigen::MatrixBase<MatrixDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Constructs from an Eigen rotation object, including Quaternion and AngleAxis.
     */
    template <typename RDerived>
    explicit AngleAxisRotation(const Eigen::RotationBase<RDerived, 3> &r)
        : Storage{typename Storage::init_storage{}, r.derived()} {}
};

namespace internal {

template <typename ImplType>
struct traits<AngleAxisRotation<ImplType>>
  : rotation_leaf_traits_base<AngleAxisRotation<ImplType>> {
    using typename rotation_leaf_traits_base<AngleAxisRotation<ImplType>>::Scalar;
    using PlainType = AngleAxisRotation<Eigen::AngleAxis<Scalar>>;
    using ConvertTo = tmp::type_list<MatrixRotation<Eigen::Matrix<Scalar, 3, 3>>,
                                     QuaternionRotation<Eigen::Quaternion<Scalar>>>;
};

/** Implements inverse of an angle-axis rotation */
template <typename Rhs>
auto evalImpl(expr<Inverse>, const AngleAxisRotation<Rhs> &a) {
    // Use Eigen's implementation
    return makeLeaf<AngleAxisRotation>(a.value().inverse());
}

/** Jacobian of inverse of an angle-axis */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<Inverse>,
                  const AngleAxisRotation<Val> &q_inv,
                  const AngleAxisRotation<Rhs> &)
  -> jacobian_t<AngleAxisRotation<Val>, AngleAxisRotation<Rhs>> {
    return -q_inv.value().toRotationMatrix();
}

// These operations are knowingly not implemented for AngleAxisRotation. It will be
// automatically converted to rotation matrix:
//   - LogMap
//   - Compose
//   - Rotate

/** Implements "conversion" between AngleAxisRotation types
 *
 * While this seems trivial, it is needed for the case the template params are not the
 * same.
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, AngleAxisRotation<ToImpl>>,
              const AngleAxisRotation<FromImpl> &rhs) {
    return AngleAxisRotation<ToImpl>{rhs.value()};
}

/** Converts from angle-axis to rotation matrix
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, MatrixRotation<ToImpl>>,
              const AngleAxisRotation<FromImpl> &rhs) {
    // Use Eigen's implementation
    return MatrixRotation<ToImpl>{rhs.value().toRotationMatrix()};
}

/** Converts from rotation matrix to angle-axis
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, AngleAxisRotation<ToImpl>>,
              const MatrixRotation<FromImpl> &rhs) {
    // Use Eigen's implementation
    return AngleAxisRotation<ToImpl>{rhs.value()};
}

/** Converts from angle-axis to quaternion
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, QuaternionRotation<ToImpl>>,
              const AngleAxisRotation<FromImpl> &rhs) {
    // Use Eigen's implementation
    return QuaternionRotation<ToImpl>{rhs.value().toRotationMatrix()};
}

/** Converts from quaternion to angle-axis
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, AngleAxisRotation<ToImpl>>,
              const QuaternionRotation<FromImpl> &rhs) {
    // Use Eigen's implementation
    return AngleAxisRotation<ToImpl>{rhs.value()};
}


}  // namespace internal

// Convenience typedefs

using RotationAd = AngleAxisRotation<Eigen::AngleAxisd>;

template <typename F1, typename F2>
using RotationAFd = Framed<RotationAd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_ANGLEAXISROTATION_HPP
