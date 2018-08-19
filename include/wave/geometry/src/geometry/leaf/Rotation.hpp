/**
 * @file
 */

#ifndef WAVE_GEOMETRY_ROTATION_HPP
#define WAVE_GEOMETRY_ROTATION_HPP

namespace wave {

namespace internal {

template <typename ImplType, typename ToFrame_, typename FromFrame_>
struct traits<MatrixRotation<ImplType, ToFrame_, FromFrame_>> {
    using ToFrame = ToFrame_;
    using FromFrame = FromFrame_;
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Archetype = Rotation<Eigen::Matrix<Scalar, 3, 3>, ToFrame, FromFrame>;
};

template <typename ImplType, typename ToFrame_, typename FromFrame_>
struct traits<QuaternionRotation<ImplType, ToFrame_, FromFrame_>> {
    using ToFrame = ToFrame_;
    using FromFrame = FromFrame_;
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Archetype = Rotation<Eigen::Quaternion<Scalar>, ToFrame, FromFrame>;
};

template <typename ImplType, typename ToFrame_, typename FromFrame_>
struct traits<AngleAxisRotation<ImplType, ToFrame_, FromFrame_>> {
    using ToFrame = ToFrame_;
    using FromFrame = FromFrame_;
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Archetype = Rotation<Eigen::AngleAxis<Scalar>, ToFrame, FromFrame>;
};

}  // namespace internal

/** A rotation stored as a rotation matrix */
template <typename ImplType, typename ToFrame, typename FromFrame>
class MatrixRotation
  : public RotationBase<MatrixRotation<ImplType, ToFrame, FromFrame>>,
    public LeafExpression<MatrixRotation<ImplType, ToFrame, FromFrame>> {
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using TangentType = RelativeRotation<Eigen::Matrix<Scalar, 3, 1>>;

 public:
    // Inherit constructors from LeafExpression
    using LeafExpression<MatrixRotation<ImplType, ToFrame, FromFrame>>::LeafExpression;

    /** Returns true if approximately equal to another matrix rotation */
    template <typename RhsImplType, typename RhsF1, typename RhsF2>
    bool isApprox(const MatrixRotation<RhsImplType, RhsF1, RhsF2> &rhs,
                  const Real &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        return this->value().isApprox(rhs.value(), prec);
    }

    /** Returns true if approximately equal to a quaternion rotation.
     * The other rotation is first converted to a matrix. */
    template <typename RhsImplType>
    bool isApprox(const QuaternionRotation<RhsImplType> &rhs,
                  const Real &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        return this->isApprox(
          typename internal::traits<MatrixRotation>::Archetype{rhs.value()}, prec);
    }

    // @todo angle-axis isApprox

    /** Check if the given matrix is a valid rotation (use when debugging only) */
    static void checkValid(const Eigen::MatrixBase<ImplType> &m) {
        // Arbitrary tolerance! Eigen uses it in isUnitary for checking the norm of each
        // column (should be 1) and the dot product of pairs of columns (should be 0), but
        // it is not a linear tolerance.
        const auto tolerance = 1e-6;
        // Note all orthogonal matrices are unitary, and we assume Scalar is real.
        if (!m.isUnitary(tolerance)) {
            throw InvalidValueError{"Rotation matrix is not orthogonal"};
        }
    }

    /* Set from the exponential map of the given so(3) relative rotation */
    template <typename TangentDerived>
    MatrixRotation &setFromExpMap(const RelativeRotationBase<TangentDerived> &rel) {
        // Using Eigen::AngleAxis is easy but maybe not optimized. @todo optimize
        const auto aa =
          Eigen::AngleAxis<Scalar>{rel.derived().angle(), rel.derived().axis()};
        this->value() = aa.toRotationMatrix();
        return *this;
    }

    /** Make a rotation from the exponential map of the given so(3) relative rotation */
    template <typename TangentDerived>
    static MatrixRotation FromExpMap(const RelativeRotationBase<TangentDerived> &rel) {
        MatrixRotation r;
        r.setFromExpMap(rel);
        return r;
    }
};

/** A rotation stored as a quaternion */
template <typename ImplType, typename ToFrame, typename FromFrame>
class QuaternionRotation
  : public RotationBase<QuaternionRotation<ImplType, ToFrame, FromFrame>>,
    public LeafExpression<QuaternionRotation<ImplType, ToFrame, FromFrame>> {
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using TangentType = RelativeRotation<Eigen::Matrix<Scalar, 3, 1>>;

 public:
    // Inherit constructors from LeafExpression
    using LeafExpression<
      QuaternionRotation<ImplType, ToFrame, FromFrame>>::LeafExpression;

    /** Returns true if approximately equal to another quaternion rotation
     * This function takes into account the two equivalent quaternion representations of
     * the same rotation.
     */
    template <typename RhsImplType>
    bool isApprox(const QuaternionRotation<RhsImplType> &rhs,
                  const Real &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        if (std::signbit(this->value().w()) == std::signbit(rhs.value().w())) {
            return this->value().coeffs().isApprox(rhs.value().coeffs(), prec);
        } else {
            return this->value().coeffs().isApprox(-rhs.value().coeffs(), prec);
        }
    }

    /** Check if the given quaternion is a valid rotation (use when debugging only) */
    template <typename ImplDerived>
    static void checkValid(const Eigen::QuaternionBase<ImplDerived> &q) {
        const auto tolerance = 1e-6;
        // Note all orthogonal matrices are unitary, and we assume Scalar is real.
        if (std::abs(q.squaredNorm() - 1) > tolerance) {
            throw InvalidValueError{"Quaternion is not normalized"};
        };
    }

    /* Set from the exponential map of the given so(3) relative rotation */
    template <typename TangentDerived>
    QuaternionRotation &setFromExpMap(const RelativeRotationBase<TangentDerived> &rel) {
        // Using Eigen::AngleAxis is easy but may make extra temporaries. @todo optimize
        const auto aa =
          Eigen::AngleAxis<Scalar>{rel.derived().angle(), rel.derived().axis()};
        this->value() = aa;
        return *this;
    }

    /** Make a rotation from the exponential map of the given so(3) relative rotation */
    template <typename TangentDerived>
    static QuaternionRotation FromExpMap(
      const RelativeRotationBase<TangentDerived> &rel) {
        return QuaternionRotation{
          Eigen::AngleAxis<Scalar>{rel.derived().angle(), rel.derived().axis()}};
    }
};


/** A rotation stored as an axis of rotation and magnitude */
template <typename ImplType, typename ToFrame, typename FromFrame>
class AngleAxisRotation
  : public RotationBase<AngleAxisRotation<ImplType, ToFrame, FromFrame>>,
    public LeafExpression<AngleAxisRotation<ImplType, ToFrame, FromFrame>> {
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using TangentType = RelativeRotation<Eigen::Matrix<Scalar, 3, 1>>;

 public:
    // Inherit constructors from LeafExpression
    using LeafExpression<AngleAxisRotation<ImplType, ToFrame, FromFrame>>::LeafExpression;

    /** Returns true if approximately equal to another angle-axis rotation
     * @warning this function considers representations with opposite signs equal, but
     * considers a rotation incremented by 2*pi different.
     */
    template <typename RhsImplType>
    bool isApprox(const AngleAxisRotation<RhsImplType> &rhs,
                  const Real &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        if (std::signbit(this->value().angle()) == std::signbit(rhs.value().angle())) {
            return this->value().isApprox(rhs.value(), prec);
        } else {
            using AAType = typename internal::impl_traits<RhsImplType>::Archetype;
            return this->value().isApprox(
              AAType{-rhs.value().angle(), -rhs.value().axis()}, prec);
        }
    }

    /** Check if the given AA is a valid rotation (use when debugging only) */
    static void checkValid(const ImplType &a) {
        if (!Eigen::internal::isApprox(typename ImplType::Scalar{1.0}, a.axis().norm())) {
            throw InvalidValueError{"Axis vector is not normalized"};
        }
        if (!std::isfinite(a.angle())) {
            throw InvalidValueError{"Angle is NaN or infinity"};
        }
    }

    /* Set from the exponential map of the given so(3) relative rotation */
    template <typename TangentDerived>
    AngleAxisRotation &setFromExpMap(const RelativeRotationBase<TangentDerived> &rel) {
        this->value() =
          Eigen::AngleAxis<Scalar>{rel.derived().angle(), rel.derived().axis()};
        return *this;
    }

    /** Make a rotation from the exponential map of the given so(3) relative rotation */
    template <typename TangentDerived>
    static AngleAxisRotation FromExpMap(const RelativeRotationBase<TangentDerived> &rel) {
        return AngleAxisRotation{rel.derived().angle(), rel.derived().axis()};
    }
};

// Convenience typedefs

/** Rotation stored as a double-precision quaternion */
using RotationQd = QuaternionRotation<Eigen::Quaterniond>;
/** Rotation stored as a double-precision quaternion, framed */
template <typename ToFrame, typename FromFrame>
using RotationQFd = QuaternionRotation<Eigen::Quaterniond, ToFrame, FromFrame>;
/** Rotation stored as a double-precision matrix */
using RotationMd = MatrixRotation<Eigen::Matrix3d>;
/** Rotation stored as a double-precision matrix, framed */
template <typename ToFrame, typename FromFrame>
using RotationMFd = MatrixRotation<Eigen::Matrix3d, ToFrame, FromFrame>;
/** Rotation stored as a double-precision angle and axis */
using RotationAd = AngleAxisRotation<Eigen::AngleAxisd>;
/** Rotation stored as a double-precision angle and axis, framed */
template <typename ToFrame, typename FromFrame>
using RotationAFd = AngleAxisRotation<Eigen::AngleAxisd, ToFrame, FromFrame>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_ROTATION_HPP
