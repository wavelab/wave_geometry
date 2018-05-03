/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_SCALAREXPRESSION_HPP
#define WAVE_GEOMETRY_SCALAREXPRESSION_HPP

namespace wave {

/** An expression representing a scalar */
template <typename Derived>
struct ScalarExpression : public ExpressionBase<Derived> {
 private:
    using Scalar = internal::scalar_t<Derived>;

 public:
    template <typename T>
    using BaseTmpl = ScalarExpression<T>;

    /** Implicitly convert to the scalar type */
    operator Scalar() const {
        return this->eval();
    }
};


namespace internal {
// Implementations of operations on scalars
// As silly as this may look, we need these operations defined in case, for example,
// someone adds two differentiable Norm expressions

template <typename Lhs,
          typename Rhs,
          TICK_REQUIRES(std::is_arithmetic<Lhs>{}, std::is_arithmetic<Rhs>{})>
auto evalImpl(expr<Sum>, const Lhs &lhs, const Rhs &rhs) -> decltype(lhs + rhs) {
    return lhs + rhs;
};

template <typename Rhs, TICK_REQUIRES(std::is_arithmetic<Rhs>{})>
auto evalImpl(expr<Minus>, const Rhs &rhs) -> decltype(-rhs) {
    return -rhs;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_SCALAREXPRESSION_HPP
