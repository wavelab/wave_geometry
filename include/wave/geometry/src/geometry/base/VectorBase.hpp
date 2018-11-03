/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_VECTORBASE_HPP
#define WAVE_GEOMETRY_VECTORBASE_HPP

namespace wave {

/** Represents a Euclidean vector
 *
 * Conceptual base class for expressions representing a Euclidean vector on
 * @f[ \mathbb{R}^n @f].
 *
 * Where A is AffineBase and V is a VectorBase, we hold
 *   V + V -> V
 *     -V  -> V
 *   A + V -> A
 *   A - A -> V
 */
template <typename Derived>
struct VectorBase : AffineBase<Derived> {
 private:
    using Scalar = internal::scalar_t<Derived>;
    using OutputType = internal::plain_output_t<Derived>;

 public:
    /** Returns a differentiable expression for the vector's L2 norm */
    auto norm() const & {
        return Norm<internal::cr_arg_t<Derived>>{this->derived()};
    }

    WAVE_OVERLOAD_SELF_METHOD_FOR_RVALUE(norm, Norm, Derived)

    /** Returns a differentiable expression for the vector's squared L2 norm */
    auto squaredNorm() const & {
        return SquaredNorm<internal::cr_arg_t<Derived>>{this->derived()};
    }

    WAVE_OVERLOAD_SELF_METHOD_FOR_RVALUE(squaredNorm, SquaredNorm, Derived)

    /** Returns a differentiable expression for this vector divided by its L2 norm */
    auto normalized() const & {
        return Normalize<internal::cr_arg_t<Derived>>{this->derived()};
    }

    WAVE_OVERLOAD_SELF_METHOD_FOR_RVALUE(normalized, Normalize, Derived)
};

namespace internal {

/** Base for traits of a vector or affine expression using an Eigen vector type for
 * storage.*/
template <typename Derived>
struct vector_leaf_traits_base;

template <template <typename...> class Tmpl, typename ImplType_>
struct vector_leaf_traits_base<Tmpl<ImplType_>> : leaf_traits_base<Tmpl<ImplType_>>,
                                                  frameable_vector_traits {
 private:
    using Derived = Tmpl<ImplType_>;

 public:
    // Vector-specific traits
    using TangentType = Derived;  // must be changed for non-vector affine objects
    using TangentBlocks = std::tuple<Derived>;  // must be changed for non-vector affine
    using ImplType = ImplType_;

    template <typename NewImplType>
    using rebind = Tmpl<NewImplType>;

    // Overrides of universal leaf traits
    using Scalar = typename ImplType::Scalar;
    using PlainType = Tmpl<typename ImplType::PlainObject>;
    enum : int { Size = ImplType::SizeAtCompileTime, TangentSize = Size };
};

/** Helper to construct a vector expression given a leaf of the same kind */
template <typename OtherLeaf, typename ImplType>
auto makeVectorLike(ImplType &&arg) {
    return typename traits<OtherLeaf>::template rebind<tmp::remove_cr_t<ImplType>>{
      std::forward<ImplType>(arg)};
}

/** Implementation of Sum for two vector leaves
 * (disambiguates overload resolution of AffineBase sums)
 * Assume spaces have already been checked.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Sum>, const VectorBase<Lhs> &lhs, const VectorBase<Rhs> &rhs) {
    return plain_output_t<Lhs>{lhs.derived().value() + rhs.derived().value()};
}

/** Implementation of difference between affine and vector leaf
 *
 * Assume spaces have already been checked.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Subtract>, const AffineBase<Lhs> &lhs, const VectorBase<Rhs> &rhs) {
    return plain_output_t<Lhs>{lhs.derived().value() - rhs.derived().value()};
}

/** Implementation of Minus for a vector leaf
 */
template <typename Rhs>
auto evalImpl(expr<Minus>, const VectorBase<Rhs> &rhs) {
    return makeVectorLike<Rhs>(-rhs.derived().value());
}

/** Implementation of squared L2 norm for a vector leaf
 */
template <typename Rhs>
auto evalImpl(expr<SquaredNorm>, const VectorBase<Rhs> &rhs) {
    return makeScalarResult(rhs.derived().value().squaredNorm());
}

/** Gradient of squared L2 norm */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<SquaredNorm>, const Val &, const VectorBase<Rhs> &rhs)
  -> jacobian_t<Val, Rhs> {
    return 2 * rhs.derived().value();
}

/** Implementation of L2 norm for a vector leaf
 */
template <typename Rhs>
auto evalImpl(expr<Norm>, const VectorBase<Rhs> &rhs) {
    return makeScalarResult(rhs.derived().value().norm());
}

/** Gradient of L2 norm */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<Norm>, const ScalarBase<Val> &norm, const VectorBase<Rhs> &rhs)
  -> jacobian_t<Val, Rhs> {
    return rhs.derived().value() / norm.derived().value();
}


/** Implementation of division by L2 norm for a vector leaf
 */
template <typename Rhs>
auto evalImpl(expr<Normalize>, const VectorBase<Rhs> &rhs) {
    return makeVectorLike<Rhs>(rhs.derived().value().normalized());
}

/** Jacobian of division by L2 norm */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<Normalize>, const VectorBase<Val> &val, const VectorBase<Rhs> &rhs)
  -> jacobian_t<Val, Rhs> {
    using J = jacobian_t<Val, Rhs>;
    /*
     * For a vector v, the derivative of v / ||v|| is, where n = ||v|| and I is identity:
     *   (I * n^2 - v*v') / n^3
     * = (I - v*v' / n^2) / n
     *
     * For example, for a vector (x, y, z) the derivative is
     * [y^2 + x^2, -xy, -xz;
     *  -xy, x^2 + z^2, -yz;
     *  -xz, -yz, y^2 + z^2] ./ norm^3
     */
    const auto &v = rhs.derived().value();
    const auto &v_over_norm = val.derived().value();
    // Matrix A = v*v' / n^2
    const auto A = v_over_norm * v_over_norm.transpose();
    return J{(J::Identity() - A) / v.norm()};
}

/** Implementation of left scalar multiplication
 * Defer to the implementation type's arithmetic operators.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Scale>, const ScalarBase<Lhs> &lhs, const VectorBase<Rhs> &rhs) {
    return makeVectorLike<Rhs>(lhs.derived().value() * rhs.derived().value());
}
/** Implementation of right scalar multiplication
 * Defer to the implementation type's arithmetic operators.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<ScaleR>, const VectorBase<Lhs> &lhs, const ScalarBase<Rhs> &rhs) {
    return makeVectorLike<Lhs>(lhs.derived().value() * rhs.derived().value());
}
/** Left Jacobian implementation for left scalar multiplication */
template <typename Res, typename Lhs, typename Rhs>
decltype(auto) leftJacobianImpl(expr<Scale>,
                                const Res &,
                                const ScalarBase<Lhs> &,
                                const VectorBase<Rhs> &rhs) {
    return rhs.derived().value();
}
/** Right Jacobian implementation for right scalar multiplication */
template <typename Res, typename Lhs, typename Rhs>
decltype(auto) rightJacobianImpl(expr<ScaleR>,
                                 const Res &,
                                 const VectorBase<Lhs> &lhs,
                                 const ScalarBase<Rhs> &) {
    return lhs.derived().value();
}
/** Left Jacobian implementation for right scalar multiplication */
template <typename Res, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<ScaleR>,
                      const Res &,
                      const VectorBase<Lhs> &,
                      const ScalarBase<Rhs> &rhs) -> jacobian_t<Res, Lhs> {
    return rhs.derived().value() * identity_t<Lhs>{};
}
/** Right Jacobian implementation for left scalar multiplication */
template <typename Res, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Scale>,
                       const Res &,
                       const ScalarBase<Lhs> &lhs,
                       const VectorBase<Rhs> &) -> jacobian_t<Res, Rhs> {
    return lhs.derived().value() * identity_t<Rhs>{};
}

/** Implementation of right scalar division
 * Defer to the implementation type's arithmetic operations.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<ScaleDiv>, const VectorBase<Lhs> &lhs, const ScalarBase<Rhs> &rhs) {
    return makeVectorLike<Lhs>(lhs.derived().value() / rhs.derived().value());
}

/** Left Jacobian implementation for right scalar division */
template <typename Res, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<ScaleDiv>,
                      const Res &,
                      const VectorBase<Lhs> &,
                      const ScalarBase<Rhs> &rhs) -> jacobian_t<Res, Lhs> {
    return identity_t<Res>{} / rhs.derived().value();
}

/** Right Jacobian implementation for right scalar division */
template <typename Res, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<ScaleDiv>,
                       const Res &,
                       const VectorBase<Lhs> &lhs,
                       const ScalarBase<Rhs> &rhs) -> jacobian_t<Res, Rhs> {
    const auto rhs_squared = rhs.derived().value() * rhs.derived().value();
    return jacobian_t<Res, Rhs>{-lhs.derived().value() / rhs_squared};
}

/** Implements dot product
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Dot>, const VectorBase<Lhs> &lhs, const VectorBase<Rhs> &rhs) {
    return makeScalarResult(lhs.derived().value().dot(rhs.derived().value()));
}

// Jacobians of dot product
template <typename Res, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<Dot>,
                      const ScalarBase<Res> &,
                      const VectorBase<Lhs> &,
                      const VectorBase<Rhs> &rhs) {
    return jacobian_t<Res, Lhs>{rhs.derived().value()};
}

template <typename Res, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Dot>,
                       const ScalarBase<Res> &,
                       const VectorBase<Lhs> &lhs,
                       const VectorBase<Rhs> &) {
    return jacobian_t<Res, Rhs>{lhs.derived().value()};
}

}  // namespace internal

/** Applies vector addition to two vector expressions (of the same space)
 *
 * @f[ \mathbb{R}^n \times \mathbb{R}^n \to \mathbb{R}^n @f]
 */
template <typename L, typename R>
auto operator+(const VectorBase<L> &lhs, const VectorBase<R> &rhs) {
    return Sum<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                             rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator+, Sum, VectorBase, VectorBase)

/** Negates a vector expression
 *
 * @f[ \mathbb{R}^n \to \mathbb{R}^n @f]
 */
template <typename R>
auto operator-(const VectorBase<R> &rhs) {
    return Minus<internal::cr_arg_t<R>>{rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUE(operator-, Minus, VectorBase)

/** Left scalar multiplication of a vector expression
 *
 * @f[ \mathbb{R} \times \mathbb{R}^n \to \mathbb{R}^n @f]
 */
template <typename L, typename R>
auto operator*(const ScalarBase<L> &lhs, const VectorBase<R> &rhs) {
    return Scale<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                               rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator*, Scale, ScalarBase, VectorBase)
WAVE_OVERLOAD_OPERATORS_FOR_SCALAR_LEFT(*, VectorBase)

/** Right scalar multiplication of a vector expression
 *
 * @f[ \mathbb{R}^n \times \mathbb{R} \to \mathbb{R}^n @f]
 */
template <typename L, typename R>
auto operator*(const VectorBase<L> &lhs, const ScalarBase<R> &rhs) {
    return ScaleR<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator*, ScaleR, VectorBase, ScalarBase)
WAVE_OVERLOAD_OPERATORS_FOR_SCALAR_RIGHT(*, VectorBase)

/** Right scalar division of a vector expression by a scalar expression
 *
 * @f[ \mathbb{R}^n \times \mathbb{R} \to \mathbb{R}^n @f]
 */
template <typename L, typename R>
auto operator/(const VectorBase<L> &lhs, const ScalarBase<R> &rhs) {
    return ScaleDiv<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                  rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator/, ScaleDiv, VectorBase, ScalarBase)
WAVE_OVERLOAD_OPERATORS_FOR_SCALAR_RIGHT(/, VectorBase)

/** Returns a differentiable expression for dot product of two vectors */
template <typename L, typename R, TICK_REQUIRES(internal::same_base_tmpl_i<L, R>{})>
auto dot(const VectorBase<L> &lhs, const VectorBase<R> &rhs) {
    return Dot<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                             rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES_REQ(
  dot, Dot, VectorBase, VectorBase, TICK_REQUIRES(internal::same_base_tmpl_i<L, R>{}))


}  // namespace wave

#endif  // WAVE_GEOMETRY_VECTORBASE_HPP
