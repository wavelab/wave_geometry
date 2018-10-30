/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_SCALARBASE_HPP
#define WAVE_GEOMETRY_SCALARBASE_HPP

namespace wave {

/** An expression representing a scalar */
template <typename Derived>
struct ScalarBase : public VectorBase<Derived> {
 private:
    using Scalar = internal::scalar_t<Derived>;

 public:
    template <typename T>
    using BaseTmpl = ScalarBase<T>;

    /** Implicitly convert to the scalar type */
    operator Scalar() const {
        // Assumes all ScalarBase are evaluated to a `Scalar` leaf
        return this->eval().value();
    }

    /** Fuzzy comparison - see Eigen::DenseBase::isApprox() */
    template <typename R, TICK_REQUIRES(internal::same_base_tmpl_i<Derived, R>{})>
    bool isApprox(
      const ScalarBase<R> &rhs,
      const Scalar &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        using std::abs;
        using std::min;
        return abs(this->eval().value() - rhs.eval().value()) <=
               prec * min(abs(this->eval().value()), abs(rhs.eval().value()));
    }

    /** True if approximately zero vector - see Eigen::DenseBase::isZero() */
    bool isZero(const Scalar &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        using std::abs;
        return abs(this->eval().value()) <= prec;
    }
};

namespace internal {

template <typename T>

/** Traits for non-expression scalar types (such as double).
 *
 * These are used for output-checking by some expressions such as Sum (which were
 * designed for vector expressions and now also serve scalars) and so we can differentiate
 * with respect to a scalar in expressions such as scalar multiplication.
 */
struct scalar_traits_base : leaf_traits_base<T>, frameable_vector_traits {
    using Scalar = T;
    enum : int { TangentSize = 1, Size = 1 };
    using TangentType = T;
    using TangentBlocks = std::tuple<T>;
};

/** Specialize traits for built-in arithmetic types.
 *
 * @note we check for non-const to avoid an ambiguous specialization. const types
 * are accepted by std::is_arithmetic but we strip the const in `traits<const T>`.
 */
template <typename T>
struct traits<T, enable_if_scalar_t<T>> : scalar_traits_base<T> {};

/** Helper to produce a scalar wrapping the input type */
template <typename Arg>
auto makeScalarResult(Arg &&arg) -> Scalar<Arg> {
    return Scalar<Arg>{std::forward<Arg>(arg)};
}

/** Trivial implementation of scalar * scalar product
 * Defer to the implementation type's arithmetic operators.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Product>, const ScalarBase<Lhs> &lhs, const ScalarBase<Rhs> &rhs) {
    return makeScalarResult(lhs.derived().value() * rhs.derived().value());
}

/** Left Jacobian implementation for scalar * scalar product
 * (return 1x1 matrix) */
template <typename Res, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<Product>,
                      const Res &,
                      const ScalarBase<Lhs> &,
                      const ScalarBase<Rhs> &rhs) -> jacobian_t<Res, Lhs> {
    return jacobian_t<Res, Lhs>{rhs.derived().value()};
}
/** Right Jacobian implementation for scalar * scalar product
 * (return 1x1 matrix) */
template <typename Res, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Product>,
                       const Res &,
                       const ScalarBase<Lhs> &lhs,
                       const ScalarBase<Rhs> &) -> jacobian_t<Res, Rhs> {
    return jacobian_t<Res, Rhs>{lhs.derived().value()};
}

/** Trivial implementation of scalar / scalar division
 * Defer to the implementation type's arithmetic operators.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Divide>, const ScalarBase<Lhs> &lhs, const ScalarBase<Rhs> &rhs) {
    return makeScalarResult(lhs.derived().value() / rhs.derived().value());
}

/** Implements arccos */
template <typename Rhs>
auto evalImpl(expr<ACos>, const ScalarBase<Rhs> &rhs) {
    using std::acos;
    return makeScalarResult(acos(rhs.derived().value()));
}

/** Jacobian of arccos */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<ACos>, const ScalarBase<Val> &, const ScalarBase<Rhs> &rhs)
  -> jacobian_t<Val, Rhs> {
    using std::sqrt;
    const auto &x = rhs.derived().value();
    // @todo blows up near x = 1
    const auto derivative = -1 / sqrt(1 - x * x);
    // Return size-1 matrix
    return jacobian_t<Val, Rhs>{derivative};
}


/** Left Jacobian implementation for scalar / scalar division
 * (return 1x1 matrix) */
template <typename Res, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<Divide>,
                      const Res &,
                      const ScalarBase<Lhs> &,
                      const ScalarBase<Rhs> &rhs) -> jacobian_t<Res, Lhs> {
    return jacobian_t<Res, Lhs>{1 / rhs.derived().value()};
}
/** Right Jacobian implementation for scalar / scalar division
 * (return 1x1 matrix) */
template <typename Res, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Divide>,
                       const Res &,
                       const ScalarBase<Lhs> &lhs,
                       const ScalarBase<Rhs> &rhs) -> jacobian_t<Res, Rhs> {
    const auto rhs_squared = rhs.derived().value() * rhs.derived().value();
    return jacobian_t<Res, Rhs>{-lhs.derived().value() / rhs_squared};
}

}  // namespace internal

/** Adds a plain scalar and a scalar expression
 *
 * @f[ \mathbb{R} \times \mathbb{R} \to \mathbb{R} @f]
 */
WAVE_OVERLOAD_OPERATORS_FOR_SCALAR(+, ScalarBase)


/** Subtracts a plain scalar and a scalar expression
 *
 * @f[ \mathbb{R} \times \mathbb{R} \to \mathbb{R} @f]
 */
WAVE_OVERLOAD_OPERATORS_FOR_SCALAR(-, ScalarBase)

/** Multiplication of two scalar expressions
 *
 * @f[ \mathbb{R} \times \mathbb{R} \to \mathbb{R} @f]
 */
template <typename L, typename R>
auto operator*(const ScalarBase<L> &lhs, const ScalarBase<R> &rhs) {
    return Product<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                 rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator*, Product, ScalarBase, ScalarBase)
WAVE_OVERLOAD_OPERATORS_FOR_SCALAR(*, ScalarBase)

/** Division of two scalar expressions
 *
 * @f[ \mathbb{R} \times \mathbb{R} \to \mathbb{R} @f]
 */
template <typename L, typename R>
auto operator/(const ScalarBase<L> &lhs, const ScalarBase<R> &rhs) {
    return Divide<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator/, Divide, ScalarBase, ScalarBase)
WAVE_OVERLOAD_OPERATORS_FOR_SCALAR(/, ScalarBase)

/** Take arccos of a scalar expression */
template <typename R>
auto acos(const ScalarBase<R> &rhs) {
    return ACos<internal::cr_arg_t<R>>{rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUE(acos, ACos, ScalarBase)

}  // namespace wave

#endif  // WAVE_GEOMETRY_SCALARBASE_HPP
