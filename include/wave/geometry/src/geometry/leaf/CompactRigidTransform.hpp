/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_COMPACTRIGIDTRANSFORM_HPP
#define WAVE_GEOMETRY_COMPACTRIGIDTRANSFORM_HPP

namespace wave {

/** A proper rigid transformation in SE(3), stored as a 7-vector
 *
 * The first four coefficients are a quaternion, stored in Eigen::Quaternion order. The
 * last three coefficients are a translation. The memory is contiguous.
 *
 * @tparam QuatType The underlying type of quaternion (e.g. Eigen::Quaterniond)
 * @tparam VecType The underlying type of translation (e.g. Eigen::Vector3d)
 * The alias RigidTransformQd is provided for the typical scalar type, double.
 */
template <typename QuatType, typename VecType>
class CompactRigidTransform
    : public RigidTransformBase<CompactRigidTransform<QuatType, VecType>>,
      public CompoundLeafStorage<CompactRigidTransform<QuatType, VecType>,
                                 QuaternionRotation<QuatType>,
                                 Translation<VecType>> {
    using Storage = CompoundLeafStorage<CompactRigidTransform<QuatType, VecType>,
                                        QuaternionRotation<QuatType>,
                                        Translation<VecType>>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    /** Constructs an uninitialized RT */
    CompactRigidTransform() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(CompactRigidTransform)

    /** Constructs from rotation and translation expressions */
    template <typename RDerived, typename TDerived>
    CompactRigidTransform(const RotationBase<RDerived> &R,
                          const TranslationBase<TDerived> &t) {
        this->rotationBlock() = R.derived();
        this->translationBlock() = t.derived();
    }

    /** Constructs from a rotation and translation, given as Eigen matrices */
    template <typename RDerived, typename TDerived>
    CompactRigidTransform(const Eigen::MatrixBase<RDerived> &R,
                          const Eigen::MatrixBase<TDerived> &t)
        : Storage{QuatType{R.derived()}, t.derived()} {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(RDerived, 3, 3);
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(TDerived, 3);
    }

    /** Construct from an Eigen rotation and translation vector */
    template <typename RDerived, typename TDerived>
    CompactRigidTransform(const Eigen::RotationBase<RDerived, 3> &q,
                          const Eigen::MatrixBase<TDerived> &t)
        : CompactRigidTransform{q, t} {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(TDerived, 3);
    };

    // block getters - for differentiable expressions and consistent interface, use
    // .rotation() and .translation()
    // @todo make private?

    /** Returns a reference to the rotation portion of this transform */
    auto &rotationBlock() noexcept {
        return std::get<0>(this->value());
    }

    /** Returns a const reference to the rotation portion of this transform */
    const auto &rotationBlock() const noexcept {
        return std::get<0>(this->value());
    }

    /** Returns a reference to the translation portion of this transform */
    auto &translationBlock() noexcept {
        return std::get<1>(this->value());
    }

    /** Returns a const reference to the translation portion of this transform */
    const auto &translationBlock() const noexcept {
        return std::get<1>(this->value());
    }
};

namespace internal {

template <typename Q, typename V>
struct traits<CompactRigidTransform<Q, V>>
    : compound_leaf_traits_base<CompactRigidTransform<Q, V>,
                                QuaternionRotation<Q>,
                                Translation<V>>,
      frameable_transform_traits {
 private:
    using TraitsBase = compound_leaf_traits_base<CompactRigidTransform<Q, V>,
                                                 QuaternionRotation<Q>,
                                                 Translation<V>>;

 public:
    using TangentType = Twist<Eigen::Matrix<typename TraitsBase::Scalar, 6, 1>>;
};

/** Converts from compact to matrix rigid transform
 */
template <typename ToImpl, typename QuatType, typename VecType>
auto evalImpl(expr<Convert, MatrixRigidTransform<ToImpl>>,
              const CompactRigidTransform<QuatType, VecType> &rhs) {
    return MatrixRigidTransform<ToImpl>{rhs.rotationBlock(), rhs.translationBlock()};
}

/** Converts from rotation matrix to quaternion
 */
template <typename QuatType, typename VecType, typename FromImpl>
auto evalImpl(expr<Convert, CompactRigidTransform<QuatType, VecType>>,
              const MatrixRigidTransform<FromImpl> &rhs) {
    return CompactRigidTransform<QuatType, VecType>{rhs.rotationBlock(),
                                                    rhs.translationBlock()};
}

/** Implements "conversion" between CompactRigidTransform types
 *
 * While this seems trivial, it is needed for the case the template params are not the
 * same.
 */
template <typename ToQ, typename ToV, typename FromQ, typename FromV>
auto evalImpl(expr<Convert, CompactRigidTransform<ToQ, ToV>>,
              const CompactRigidTransform<FromQ, FromV> &rhs) {
    return CompactRigidTransform<ToQ, ToV>{rhs.rotationBlock(), rhs.translationBlock()};
}


}  // namespace internal

// Convenience typedefs

using RigidTransformQd = CompactRigidTransform<Eigen::Quaterniond, Eigen::Vector3d>;

template <typename F1, typename F2>
using RigidTransformQFd = Framed<RigidTransformQd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_COMPACTRIGIDTRANSFORM_HPP
