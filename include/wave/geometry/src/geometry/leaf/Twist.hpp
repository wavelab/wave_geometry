/**
 * @file
 */

#ifndef WAVE_GEOMETRY_TWIST_HPP
#define WAVE_GEOMETRY_TWIST_HPP

namespace wave {

/** A difference between rigid transforms, with its own storage
 *
 * A Twist is more formally an element of @f$ se(3) @f$, the Lie algebra of the Lie group
 * @f$ SE(3) @f$. It can represent a small rotation and translation or a derivative of a
 * rigid transform. It is to SE(3) what RelativeRotation is to SO(3).
 *
 * Twist is parameterized as a 6-vector whose first three coefficients are a Translation,
 * and last three coefficients are a RelativeRotation.
 *
 * @tparam ImplType The type to use for storage
 *
 * The alias Twistd is provided for the typical storage type, Eigen::Matrix<double, 6, 1>.
 */
template <typename ImplType>
class Twist : public TwistBase<Twist<ImplType>>,
              public LeafStorage<Twist<ImplType>, ImplType> {
    static_assert(internal::is_eigen_vector<6, ImplType>::value,
                  "ImplType must be an Eigen 6-vector type.");
    using Scalar = typename ImplType::Scalar;
    using Storage = LeafStorage<Twist<ImplType>, ImplType>;

    using Block = Eigen::Block<ImplType, 3, 1>;
    using ConstBlock = Eigen::Block<const ImplType, 3, 1>;


 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    /** Construct an uninitialized Twist */
    Twist() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(Twist)

    /** Construct from Eigen 6-vector expression (rotation first, then translation) */
    template <typename VDerived>
    Twist(const Eigen::MatrixBase<VDerived> &v)
        : Storage{typename Storage::init_storage{}, v.derived()} {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(VDerived, 6);
    }

    /** Construct from a relative rotation and translation, given as Eigen matrices */
    template <typename RDerived, typename TDerived>
    Twist(const Eigen::MatrixBase<RDerived> &rr, const Eigen::MatrixBase<TDerived> &t) {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(RDerived, 3);
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(TDerived, 3);
        this->storage << rr, t;
    };

    /** Returns a reference to the rotation portion of this twist */
    RelativeRotation<Block> rotation() noexcept {
        return RelativeRotation<Block>{this->value().template head<3>()};
    }

    /** Returns a const reference to the rotation portion of this twist */
    RelativeRotation<ConstBlock> rotation() const noexcept {
        return RelativeRotation<ConstBlock>{this->value().template head<3>()};
    }

    /** Returns a reference to the translation portion of this twist */
    Translation<Block> translation() noexcept {
        return Translation<Block>{this->value().template tail<3>()};
    }

    /** Returns a const reference to the translation portion of this twist */
    Translation<ConstBlock> translation() const noexcept {
        return Translation<ConstBlock>{this->value().template tail<3>()};
    }
};

namespace internal {

template <typename ImplType>
struct traits<Twist<ImplType>> : vector_leaf_traits_base<Twist<ImplType>> {
    using ExpType = MatrixRigidTransform<Eigen::Matrix<typename ImplType::Scalar, 4, 4>>;
};

/** Implements exp map of a twist into a MatrixRigidTransform
 *
 * @todo - evaluate to either
 */
template <typename ImplType>
auto evalImpl(expr<ExpMap>, const Twist<ImplType> &rhs) ->
  typename traits<Twist<ImplType>>::ExpType {
    using Scalar = typename ImplType::Scalar;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

    // For now, calculate expmap in two parts
    typename traits<Twist<ImplType>>::ExpType out{};

    // Equations: see http://ethaneade.com/lie.pdf
    // @todo optimize
    const auto &omega = rhs.rotation().value();  // the rotation part
    const auto theta2 = omega.squaredNorm();
    const auto theta = std::sqrt(theta2);
    if (theta2 > Eigen::NumTraits<Scalar>::epsilon()) {
        const auto A = std::sin(theta) / theta;
        const auto B = (Scalar{1.0} - std::cos(theta)) / theta2;
        const auto C = (Scalar{1.0} - A) / theta2;
        const Mat3 cross = crossMatrix(omega);
        const Mat3 cross2 = cross * cross;
        const Mat3 V = Mat3::Identity() + B * cross + C * cross2;
        out.translation().value() = V * rhs.translation().value();
        out.rotation().value() = Mat3::Identity() + A * cross + B * cross2;
    } else {
        // small theta2; use Taylor expansions
        const auto A = Scalar{1.0};
        const auto B = Scalar{0.0};
        const auto C = Scalar{0.0};
        const Mat3 cross = crossMatrix(omega);
        const Mat3 cross2 = cross * cross;
        const Mat3 V = Mat3::Identity() + B * cross + C * cross2;
        out.translation().value() = V * rhs.translation().value();
        out.rotation().value() = Mat3::Identity() + A * cross + B * cross2;
    }

    return out;
}

/** Jacobian of ExpMap for a twist */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<ExpMap>, const TransformBase<Val> &val, const TwistBase<Rhs> &rhs)
  -> jacobian_t<Val, Rhs> {
    using Scalar = scalar_t<Val>;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

    // From http://ethaneade.org/exp_diff.pdf - note we swap order of rotation and
    // translation
    using std::cos;
    using std::sin;
    // First get Jacobian of expmap of rotation part only
    const auto &Drot =
      jacobianImpl(expr<ExpMap>{}, val.derived().rotation(), rhs.derived().rotation());

    const auto &omega = rhs.derived().rotation().value();
    const auto &u = rhs.derived().translation().value();
    const auto theta2 = omega.squaredNorm();
    const auto theta = sqrt(theta2);
    const auto a = sin(theta) / theta;
    const auto b = (1 - cos(theta)) / theta2;
    const auto c = (1 - a) / theta2;
    // Calculate Eade's "W" term
    const Mat3 W = (c - b) * Mat3::Identity() +
                   (a - 2 * b) / theta2 * crossMatrix(omega) +
                   (b - 3 * c) / theta2 * omega * omega.transpose();

    // Calculate Eade's "B" term
    const Mat3 B = b * crossMatrix(u) +
                   c * (omega * u.transpose() + u * omega.transpose()) + omega.dot(u) * W;


    Eigen::Matrix<Scalar, 6, 6> out{};
    out.template topLeftCorner<3, 3>() = Drot;
    out.template bottomLeftCorner<3, 3>() = B;
    out.template topRightCorner<3, 3>().setZero();
    out.template bottomRightCorner<3, 3>() = Drot;
    return out;
}

}  // namespace internal

// Convenience typedefs

using Twistd = Twist<Eigen::Matrix<double, 6, 1>>;

template <typename F1, typename F2, typename F3>
using TwistFd = Framed<Twistd, F1, F2, F3>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_TWIST_HPP
