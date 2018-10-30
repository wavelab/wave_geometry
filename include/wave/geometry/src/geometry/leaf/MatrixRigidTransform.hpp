/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_MATRIXRIGIDTRANSFORM_HPP
#define WAVE_GEOMETRY_MATRIXRIGIDTRANSFORM_HPP

namespace wave {

/** A proper rigid transformation in SE(3), stored as a 4x4 matrix
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Matrix4d or a Map)
 *
 * The alias RigidTransformMd is provided for the typical storage type, Eigen::Matrix4d.
 */
template <typename ImplType>
class MatrixRigidTransform
    : public RigidTransformBase<MatrixRigidTransform<ImplType>>,
      public LeafStorage<MatrixRigidTransform<ImplType>, ImplType> {
    static_assert(internal::is_eigen_matrix<4, ImplType>::value,
                  "ImplType must be an Eigen 4x4 matrix type.");

    using Storage = LeafStorage<MatrixRigidTransform<ImplType>, ImplType>;
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;

    // Eigen Blocks for nested rotation and translation
    using RotationBlock = Eigen::Block<ImplType, 3, 3>;
    using RotationConstBlock = Eigen::Block<const ImplType, 3, 3>;
    using TranslationBlock = Eigen::Block<ImplType, 3, 1>;
    using TranslationConstBlock = Eigen::Block<const ImplType, 3, 1>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    /** Construct an uninitialized RT, except fill the last row */
    MatrixRigidTransform() {
        this->value().row(3) << Scalar{0}, Scalar{0}, Scalar{0}, Scalar{1};
    }

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(MatrixRigidTransform)

    /** Constructs from rotation and translation expressions */
    template <typename RDerived, typename TDerived>
    MatrixRigidTransform(const RotationBase<RDerived> &R,
                         const TranslationBase<TDerived> &t)
        : MatrixRigidTransform{} {
        this->rotationBlock() = R.derived();
        this->translationBlock() = t.derived();
    };

    /** Construct from a rotation and translation, given as Eigen matrices */
    template <typename RDerived, typename TDerived>
    MatrixRigidTransform(const Eigen::MatrixBase<RDerived> &R,
                         const Eigen::MatrixBase<TDerived> &t)
        : MatrixRigidTransform{} {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(RDerived, 3, 3);
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(TDerived, 3);
        this->rotationBlock().value() << R;
        this->translationBlock().value() << t;
    };

    /** Construct from an Eigen rotation and translation vector */
    template <typename RDerived, typename TDerived>
    MatrixRigidTransform(const Eigen::RotationBase<RDerived, 3> &q,
                         const Eigen::MatrixBase<TDerived> &t)
        : MatrixRigidTransform{q.toRotationMatrix(), t} {};

    /** Construct from an Eigen 4x4 transformation matrix */
    template <typename MDerived, TICK_REQUIRES(internal::is_eigen_matrix<4, MDerived>{})>
    explicit MatrixRigidTransform(const Eigen::MatrixBase<MDerived> &m)
        : Storage{typename Storage::init_storage{}, m} {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(MDerived, 4, 4);
    }

    // block getters - for differentiable expressions and consistent interface, use
    // .rotation() and .translation()
    // @todo make private?

    /** Returns a mutable expression referring to the translation portion of this
     * transform */
    auto rotationBlock() noexcept -> MatrixRotation<RotationBlock> {
        return MatrixRotation<RotationBlock>{
          this->value().template topLeftCorner<3, 3>()};
    }

    /** Returns an expression referring to the rotation portion of this transform */
    auto rotationBlock() const noexcept -> MatrixRotation<RotationConstBlock> {
        return MatrixRotation<RotationConstBlock>{
          this->value().template topLeftCorner<3, 3>()};
    }

    /** Returns a mutable expression referring to the translation portion of this
     * transform */
    auto translationBlock() noexcept -> Translation<TranslationBlock> {
        return Translation<TranslationBlock>{
          this->value().template topRightCorner<3, 1>()};
    }

    /** Returns an expression referring to the translation portion of this transform */
    auto translationBlock() const noexcept -> Translation<TranslationConstBlock> {
        return Translation<TranslationConstBlock>{
          this->value().template topRightCorner<3, 1>()};
    }
};

namespace internal {

template <typename Derived>
struct rt_leaf_traits_base;

template <typename ImplType>
struct traits<MatrixRigidTransform<ImplType>>
    : rt_leaf_traits_base<MatrixRigidTransform<ImplType>> {
    using PlainType = MatrixRigidTransform<typename ImplType::PlainObject>;
};

/** Implementations Identity for a rigid transform
 *
 * @todo split up to matrix and compact RT */
template <typename Leaf, TICK_REQUIRES(tmp::is_crtp_base_of<RigidTransformBase, Leaf>{})>
auto evalImpl(expr<Convert, Leaf>, const Identity<Leaf> &) {
    using Scalar = internal::scalar_t<Leaf>;
    return Leaf{Eigen::Quaternion<Scalar>::Identity(),
                Eigen::Matrix<Scalar, 3, 1>::Zero()};
}

/** Implements "conversion" between MatrixRigidTransform types
 *
 * While this seems trivial, it is needed for the case the template params are not the
 * same.
 */
template <typename ToImpl, typename FromImpl>
auto evalImpl(expr<Convert, MatrixRigidTransform<ToImpl>>,
              const MatrixRigidTransform<FromImpl> &rhs) {
    return MatrixRigidTransform<ToImpl>{rhs.value()};
}

}  // namespace internal

// Convenience typedefs

using RigidTransformMd = MatrixRigidTransform<Eigen::Matrix4d>;

template <typename F1, typename F2>
using RigidTransformMFd = Framed<RigidTransformMd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_MATRIXRIGIDTRANSFORM_HPP
