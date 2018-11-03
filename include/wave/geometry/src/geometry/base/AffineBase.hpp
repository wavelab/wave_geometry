/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_AFFINEBASE_HPP
#define WAVE_GEOMETRY_AFFINEBASE_HPP

namespace wave {

/** Represents an affine vector or point
 *
 * Conceptual base class for expressions representing a point or vector object on an
 * affine space.
 *
 * Where A is AffineBase and V is a VectorBase, we hold
 *   A + V -> A
 *   A - V -> A
 *   A - A -> V
 */
template <typename Derived>
struct AffineBase : ExpressionBase<Derived> {
 private:
    using Scalar = internal::scalar_t<Derived>;
    using OutputType = internal::plain_output_t<Derived>;

 public:
    static auto Random() -> OutputType {
        return ::wave::Random<OutputType>{}.eval();
    }

    /** Returns an expression representing a zero element */
    static auto Zero() -> ::wave::Zero<OutputType> {
        return ::wave::Zero<OutputType>{};
    }

    /** Fuzzy comparison - see Eigen::DenseBase::isApprox() */
    template <typename R, TICK_REQUIRES(internal::same_base_tmpl_i<Derived, R>{})>
    bool isApprox(
      const VectorBase<R> &rhs,
      const Scalar &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        return this->eval().value().isApprox(rhs.eval().value(), prec);
    }

    /** True if value is approximately zero - see Eigen::DenseBase::isZero() */
    bool isZero(const Scalar &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        return this->eval().value().isZero(prec);
    }
};

namespace internal {

/** Implementation of conversion between two affine leaves
 *
 * While this seems trivial, it is needed for the case the template params are not the
 * same.
 */
template <typename ToType,
          typename Rhs,
          TICK_REQUIRES(internal::same_base_tmpl<ToType, Rhs>{})>
auto evalImpl(expr<Convert, ToType>, const AffineBase<Rhs> &rhs) -> ToType {
    return ToType{rhs.derived().value()};
}

/** Implementation of Sum for an affine and vector leaf
 *
 * Assume spaces have already been checked.
 * Since the temporary is so small, we return a plain vector instead of an Eigen::Sum.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Sum>, const AffineBase<Lhs> &lhs, const VectorBase<Rhs> &rhs) {
    return plain_output_t<Lhs>{lhs.derived().value() + rhs.derived().value()};
}

/** Implementation of Sum (flipped) for an affine and vector leaf
 *
 * Assume spaces have already been checked.
 * Since the temporary is so small, we return a plain vector instead of an Eigen::Sum.
 */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<Sum>, const VectorBase<Lhs> &lhs, const AffineBase<Rhs> &rhs) {
    return plain_output_t<Rhs>{lhs.derived().value() + rhs.derived().value()};
}

/** Implementation of Random for an affine leaf
 *
 * Produces a random vector with coefficients from -1 to 1
 */
template <typename Leaf, typename Rhs>
auto evalImpl(expr<Random, Leaf>, const AffineBase<Rhs> &) {
    return Leaf{internal::traits<Leaf>::ImplType::Random()};
}

}  // namespace internal

/** Adds an affine expression and a vector (of its difference type)
 *
 * @f[ \mathbb{R}^n \times \mathbb{R}^n \to \mathbb{R}^n @f]
 */
template <typename L, typename R>
auto operator+(const AffineBase<L> &lhs, const VectorBase<R> &rhs) {
    return Sum<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                             rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator+, Sum, AffineBase, VectorBase)

/** Adds an affine expression and a vector (of its difference type)
 *
 * @f[ \mathbb{R}^n \times \mathbb{R}^n \to \mathbb{R}^n @f]
 */
template <typename L, typename R>
auto operator+(const VectorBase<L> &lhs, const AffineBase<R> &rhs) {
    return Sum<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                             rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator+, Sum, VectorBase, AffineBase)

/** Subtracts two affine expressions
 *
 * R must be the difference type of L.
 *
 * @f[ \mathbb{R}^n \times \mathbb{R}^n \to \mathbb{R}^n @f]
 */
template <typename L, typename R>
auto operator-(const AffineBase<L> &lhs, const AffineBase<R> &rhs) {
    return lhs.derived() + (-rhs.derived());
}

// Overloads for rvalues
template <typename L, typename R>
auto operator-(AffineBase<L> &&lhs, const AffineBase<R> &rhs) {
    return std::move(lhs).derived() + (-rhs.derived());
}
template <typename L, typename R>
auto operator-(const AffineBase<L> &lhs, AffineBase<R> &&rhs) {
    return lhs.derived() + (-std::move(rhs).derived());
}
template <typename L, typename R>
auto operator-(AffineBase<L> &&lhs, AffineBase<R> &&rhs) {
    return std::move(lhs).derived() + (-std::move(rhs).derived());
}

}  // namespace wave

#endif  // WAVE_GEOMETRY_AFFINEBASE_HPP
