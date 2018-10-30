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
 * @tparam ImplType The type to use for storage (e.g. Eigen::Matrix<double, 7, 1>)
 *
 * The alias RigidTransformQd is provided for the typical scalar type, double.
 */
template <typename ImplType>
class CompactRigidTransform
    : public RigidTransformBase<CompactRigidTransform<ImplType>>,
      public LeafStorage<CompactRigidTransform<ImplType>, ImplType> {
    static_assert(internal::is_eigen_vector<7, ImplType>::value,
                  "ImplType must be an Eigen 7-vector type.");

    using Storage = LeafStorage<CompactRigidTransform<ImplType>, ImplType>;
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;

    // Eigen Blocks for nested rotation and translation
    using RotationMap = Eigen::Map<Eigen::Quaternion<Scalar>>;
    using RotationConstMap = Eigen::Map<const Eigen::Quaternion<Scalar>>;
    using TranslationBlock = Eigen::Block<ImplType, 3, 1>;
    using TranslationConstBlock = Eigen::Block<const ImplType, 3, 1>;


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
    };

    /** Constructs from a rotation and translation, given as Eigen matrices */
    template <typename RDerived, typename TDerived>
    CompactRigidTransform(const Eigen::MatrixBase<RDerived> &R,
                          const Eigen::MatrixBase<TDerived> &t) {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(RDerived, 3, 3);
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(TDerived, 3);
        this->value() << Eigen::Quaternion<Scalar>{R}.coeffs(), t;
    };

    /** Construct from an Eigen rotation and translation vector */
    template <typename RDerived, typename TDerived>
    CompactRigidTransform(const Eigen::RotationBase<RDerived, 3> &q,
                          const Eigen::MatrixBase<TDerived> &t)
        : CompactRigidTransform{q.matrix(), t} {};

    /** Construct from an Eigen 7-vector */
    template <typename VDerived, TICK_REQUIRES(internal::is_eigen_vector<7, VDerived>{})>
    explicit CompactRigidTransform(const Eigen::MatrixBase<VDerived> &v)
        : Storage{typename Storage::init_storage{}, v.derived()} {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VDerived, 7);
    }

    // block getters - for differentiable expressions and consistent interface, use
    // .rotation() and .translation()
    // @todo make private?

    /** Returns a reference to the rotation portion of this transform */
    QuaternionRotation<RotationMap> rotationBlock() noexcept {
        return QuaternionRotation<RotationMap>{RotationMap{this->value().data()}};
    }

    /** Returns a const reference to the rotation portion of this transform */
    QuaternionRotation<RotationConstMap> rotationBlock() const noexcept {
        return QuaternionRotation<RotationConstMap>{
          RotationConstMap{this->value().data()}};
    }

    /** Returns a reference to the translation portion of this transform */
    Translation<TranslationBlock> translationBlock() noexcept {
        return Translation<TranslationBlock>{this->value().template tail<3>()};
    }

    /** Returns a const reference to the translation portion of this transform */
    Translation<TranslationConstBlock> translationBlock() const noexcept {
        return Translation<TranslationConstBlock>{this->value().template tail<3>()};
    }
};

namespace internal {

template <typename ImplType>
struct traits<CompactRigidTransform<ImplType>>
    : rt_leaf_traits_base<CompactRigidTransform<ImplType>> {
    using PlainType = CompactRigidTransform<typename ImplType::PlainObject>;
};

/** Converts from compact to matrix rigid transform
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, MatrixRigidTransform<ToImpl>>,
              const CompactRigidTransform<FromImpl> &rhs) {
    return MatrixRigidTransform<ToImpl>{rhs.rotation(), rhs.translation()};
}

/** Converts from rotation matrix to quaternion
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, CompactRigidTransform<ToImpl>>,
              const MatrixRigidTransform<FromImpl> &rhs) {
    return CompactRigidTransform<ToImpl>{rhs.rotation(), rhs.translation()};
}

/** Implements "conversion" between CompactRigidTransform types
 *
 * While this seems trivial, it is needed for the case the template params are not the
 * same.
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, CompactRigidTransform<ToImpl>>,
              const CompactRigidTransform<FromImpl> &rhs) {
    return CompactRigidTransform<ToImpl>{rhs.value()};
}


}  // namespace internal

// Convenience typedefs

using RigidTransformQd = CompactRigidTransform<Eigen::Matrix<double, 7, 1>>;

template <typename F1, typename F2>
using RigidTransformQFd = Framed<RigidTransformQd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_COMPACTRIGIDTRANSFORM_HPP
