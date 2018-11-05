/**
 * @file
 */

#ifndef WAVE_GEOMETRY_UNITHOMOGENEOUSPOINT_HPP
#define WAVE_GEOMETRY_UNITHOMOGENEOUSPOINT_HPP

namespace wave {

/** A spherically normalized point in P^3, with its own storage
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Quaterniond)
 *
 * The alias UnitHomogeneousPointd is provided for the typical storage type,
 * Eigen::Quaterniond.
 */
template <typename ImplType>
class UnitHomogeneousPoint
    : public HomogeneousPointBase<UnitHomogeneousPoint<ImplType>>,
      public LeafStorage<UnitHomogeneousPoint<ImplType>, ImplType> {
    static_assert(internal::is_eigen_quaternion<ImplType>::value,
                  "ImplType must be an Eigen quaternion type.");
    using Scalar = typename ImplType::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using Storage = LeafStorage<UnitHomogeneousPoint<ImplType>, ImplType>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    UnitHomogeneousPoint() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(UnitHomogeneousPoint)

    /** Constructs from an Eigen 4-vector */
    template <typename OtherDerived>
    UnitHomogeneousPoint(const Eigen::MatrixBase<OtherDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Constructs from an Eigen quaternion object */
    template <typename QDerived>
    explicit UnitHomogeneousPoint(const Eigen::QuaternionBase<QDerived> &q)
        : Storage{typename Storage::init_storage{}, q.derived()} {}

    /** Construct from four scalars
     *
     * @note this constructor does not match Eigen::Quaternion's constructor order,
     * though it does match Eigen::Quaternion's storage order,
     */
    UnitHomogeneousPoint(Scalar x, Scalar y, Scalar z, Scalar w)
        : Storage{typename Storage::init_storage{}, w, x, y, z} {}
};

namespace internal {

template <typename ImplType_>
struct traits<UnitHomogeneousPoint<ImplType_>>
    : leaf_traits_base<UnitHomogeneousPoint<ImplType_>>, frameable_transform_traits {
    using ImplType = ImplType_;
    using Scalar = typename ImplType::Scalar;

    template <typename NewImplType>
    using rebind = UnitHomogeneousPoint<NewImplType>;

    using TangentType = Translation<Eigen::Matrix<Scalar, 3, 1>>;
    using TangentBlocks = tmp::type_list<TangentType>;
    enum : int { TangentSize = 3 };

    using PlainType =
      UnitHomogeneousPoint<typename Eigen::internal::traits<ImplType>::PlainObject>;
    using ConvertTo = tmp::type_list<HomogeneousPoint<Eigen::Matrix<Scalar, 4, 1>>>;
};

/** Converts an unnormalized HomogeneousPoint to spherically normalized
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, UnitHomogeneousPoint<ToImpl>>,
              const HomogeneousPoint<FromImpl> &rhs) {
    return UnitHomogeneousPoint<ToImpl>{rhs.value().normalized()};
}

/** Implements quasi-"inverse" for BoxMinus of spherical homogeneous points
 */
template <typename Rhs>
auto evalImpl(expr<Inverse>, const UnitHomogeneousPoint<Rhs> &rhs) {
    return makeLeaf<UnitHomogeneousPoint>(rhs.value().conjugate());
}

/** Jacobian of "inverse"  of spherical homogeneous points, for BoxMinus */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<Inverse>,
                  const UnitHomogeneousPoint<Val> &q_inv,
                  const UnitHomogeneousPoint<Rhs> &)
  -> jacobian_t<UnitHomogeneousPoint<Val>, UnitHomogeneousPoint<Rhs>> {
    return -q_inv.value().toRotationMatrix();
}

/** Implements quasi-"logmap" for BoxMinus of spherical homogeneous points
 */
template <typename Rhs>
auto evalImpl(expr<LogMap>, const UnitHomogeneousPoint<Rhs> &rhs) {
    using VType = typename traits<UnitHomogeneousPoint<Rhs>>::TangentType;
    return VType{::wave::rotationVectorFromQuaternion(rhs.value())};
}

/** Jacobian of "logmap" for BoxMinus of spherical homogeneous points
 */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<LogMap>,
                  const TranslationBase<Val> &val,
                  const HomogeneousPointBase<Rhs> &) -> jacobian_t<Val, Rhs> {
    return ::wave::jacobianOfRotationLogMap(val.derived().value());
}

/** Implements quasi-"expmap" for BoxPlus of spherical homogeneous points
 * @todo reorganize? Expmap of translation may be unexpected.
 */
template <typename ImplType>
auto evalImpl(expr<ExpMap>, const Translation<ImplType> &rhs) {
    using QType = Eigen::Quaternion<typename ImplType::Scalar>;
    return UnitHomogeneousPoint<QType>{::wave::quaternionFromRotationVector(rhs.value())};
}

/** Implements quasi-"compose" for manifold operations on spherical homogeneous points
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Compose>,
              const UnitHomogeneousPoint<Lhs> &lhs,
              const UnitHomogeneousPoint<Rhs> &rhs) {
    // Use Eigen's implementation
    return plain_eval_t<UnitHomogeneousPoint<Lhs>>{lhs.value() * rhs.value()};
}

/** Left jacobian of "compose" of spherical homogeneous points */
template <typename Val, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<Compose>,
                      const HomogeneousPointBase<Val> &,
                      const HomogeneousPointBase<Lhs> &,
                      const HomogeneousPointBase<Rhs> &) -> jacobian_t<Val, Rhs> {
    return identity_t<Val>{};
}

/** Right jacobian of "compose" of spherical homogeneous points */
template <typename Val, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Compose>,
                       const UnitHomogeneousPoint<Val> &,
                       const UnitHomogeneousPoint<Lhs> &lhs,
                       const UnitHomogeneousPoint<Rhs> &)
  -> jacobian_t<UnitHomogeneousPoint<Val>, UnitHomogeneousPoint<Rhs>> {
    return lhs.value().toRotationMatrix();
}

/** Implements manifold addition to homogeneous points
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<HomPlus>,
              const UnitHomogeneousPoint<Lhs> &lhs,
              const VectorBase<Rhs> &rhs) {
    return plain_output_t<UnitHomogeneousPoint<Lhs>>{
      ::wave::quaternionFromRotationVector(rhs.derived().value()) * lhs.value()};
}

}  // namespace internal

// Convenience typedefs

using UnitHomogeneousPointd = UnitHomogeneousPoint<Eigen::Quaterniond>;

/** A 3D point whose position is defined by by the vector TranslationFd<F1, F1, F2>.
 */
template <typename F1, typename F2>
using UnitHomogeneousPointFd = Framed<UnitHomogeneousPointd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_UNITHOMOGENEOUSPOINT_HPP
