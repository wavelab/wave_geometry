/**
 * @file
 */

#ifndef WAVE_GEOMETRY_RELATIVEROTATION_HPP
#define WAVE_GEOMETRY_RELATIVEROTATION_HPP

namespace wave {

/** A "small" rotation or difference between orientations, with its own storage
 *
 * What we call a "relative rotation" is more formally an element of @f$ so(3) @f$, the
 * Lie algebra of the Lie group @f$ SO(3) @f$. It can represent a small rotation or a
 * derivative of a rotation.
 *
 * RelativeRotation corresponds to @f$\varphi@f$ in Bloesch et al's "A Primer on the
 * Differential Calculus of 3D Orientations". It lies on the Lie algebra
 * @f$|mathbb{R}^3@f$, which is isomorphic to @f$ so(3) @f$. It is parameterized as a
 * 3-vector whose direction represents the axis of rotation, and magnitude the angle of
 * the rotation (in radians).
 *
 * This representation is minimal and unique: all values are valid relative rotations, and
 * every relative rotation maps to exactly one value. (Conversely, quaternions, matrices,
 * and Eigen's non-compact AngleAxis only represent valid rotations when normalized, and
 * quaternions and AngleAxis can represent the same rotation with multiple values).
 *
 * In this library, Jacobians of Rotation expressions are expressed in terms of this
 * parametrization.
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Vector3d or
 * Eigen::Map<Eigen::Vector3f>)
 *
 * The alias RelativeRotationd is provided for the typical storage type, Eigen::Vector3d.
 */
template <typename ImplType>
class RelativeRotation : public RelativeRotationBase<RelativeRotation<ImplType>>,
                         public LeafStorage<RelativeRotation<ImplType>, ImplType> {
    static_assert(internal::is_eigen_vector<3, ImplType>::value,
                  "ImplType must be an Eigen 3-vector type.");
    using Scalar = typename ImplType::Scalar;
    using Storage = LeafStorage<RelativeRotation<ImplType>, ImplType>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    RelativeRotation() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(RelativeRotation)

    /** Construct from Eigen Matrix object */
    template <typename OtherDerived>
    RelativeRotation(const Eigen::MatrixBase<OtherDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Construct from three scalars */
    RelativeRotation(Scalar x, Scalar y, Scalar z)
        : Storage{typename Storage::init_storage{}, x, y, z} {}

    /**
     * Set from an angle and a vector representing the axis of rotation. The axis does not
     * have to be normalized.
     */
    template <typename AxisDerived>
    RelativeRotation &setFromAngleAndAxis(Scalar angle,
                                          const Eigen::MatrixBase<AxisDerived> &axis) {
        this->value() = angle * axis.normalized();
        return *this;
    }

    /**
     * Returns a RelativeRotation representing a rotation of `angle` about `axis`.
     * The axis does not have to be normalized.
     */
    template <typename AxisDerived>
    static RelativeRotation FromAngleAndAxis(Scalar angle,
                                             const Eigen::MatrixBase<AxisDerived> &axis) {
        RelativeRotation r;
        r.setFromAngleAndAxis(angle, axis);
        return r;
    }
};

namespace internal {

template <typename ImplType>
struct traits<RelativeRotation<ImplType>>
    : vector_leaf_traits_base<RelativeRotation<ImplType>> {
    // Type of exponential map, currently used by expmap of Zero expression
    using ExpType = MatrixRotation<Eigen::Matrix<typename ImplType::Scalar, 3, 3>>;
};

/** Implements exp map of a relative rotation into a quaternion rotation
 *
 * When evaluating a rotation matrix, a conversion is needed. However, as shown by
 * expmap_bench.cpp, this method is faster than the Rodrigues formula expmap to matrix,
 * even after the conversion.
 *
 * Based on:  F. S. Grassia, "Practical parameterization of rotations using the
 * exponential map," Journal of graphics tools, 1998.
 */
template <typename ImplType>
auto evalImpl(expr<ExpMap>, const RelativeRotation<ImplType> &rhs) {
    using Scalar = typename ImplType::Scalar;
    using std::cos;
    using std::sin;
    using std::sqrt;

    auto q = QuaternionRotation<Eigen::Quaternion<Scalar>>{};
    const auto &v = rhs.value();
    const Scalar angle2 = v.squaredNorm();
    const Scalar angle = sqrt(angle2);
    Scalar s;
    Scalar c;

    if (angle2 * angle2 > Eigen::NumTraits<Scalar>::epsilon()) {
        const Scalar sa = sin(angle / 2);
        s = sa / angle;
        c = cos(angle / 2);
    } else {
        s = Scalar{0.5} + angle2 / 48;
        c = 1 - angle2 / 8;
    }

    // storage order x, y, z, w
    q.value().coeffs() << s * v, c;
    return q;
}

/** Jacobian of exp map of a relative rotation */
template <typename Val, typename ImplType>
auto jacobianImpl(expr<ExpMap>,
                  const RotationBase<Val> &val,
                  const RelativeRotation<ImplType> &rhs)
  -> jacobian_t<Val, RelativeRotation<ImplType>> {
    using Jacobian = jacobian_t<Val, RelativeRotation<ImplType>>;
    using Scalar = typename ImplType::Scalar;
    const auto &phi = rhs.value();
    const auto &C =
      val.derived().value().matrix();  // Rotation matrix of the SO(3) output

    // Bloesch Equation 80, with adjustment for near-zero case
    const auto pcross = crossMatrix(phi);  // cross-matrix of the so(3) input
    const auto n2 = phi.squaredNorm();     // squared norm of the input
    if (n2 > Eigen::NumTraits<Scalar>::epsilon()) {
        return ((Jacobian::Identity() - C) * pcross + (phi * phi.transpose())) / n2;
    } else {
        return Jacobian::Identity() + Scalar{0.5} * pcross;
    }
}

}  // namespace internal

// Convenience typedefs

using RelativeRotationd = RelativeRotation<Eigen::Vector3d>;

template <typename F1, typename F2, typename F3>
using RelativeRotationFd = Framed<RelativeRotationd, F1, F2, F3>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_RELATIVEROTATION_HPP
