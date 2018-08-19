/**
 * @file
 */

#ifndef WAVE_GEOMETRY_SUM_HPP
#define WAVE_GEOMETRY_SUM_HPP
#include <iostream>
namespace wave {

/** An expression representing the vector sum of two vector expressions
 * This expression can be applied to pairs of Translation or RelativeRotation expressions
 * */
template <typename Lhs, typename Rhs>
struct Sum : internal::base_tmpl_t<Lhs, Rhs, Sum<Lhs, Rhs>>,
             BinaryExpression<Sum<Lhs, Rhs>> {
    // Inherit constructor from BinaryExpression
    using BinaryExpression<Sum<Lhs, Rhs>>::BinaryExpression;

    // clang-format off
    // keep static_assert on one line for compiler output

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Operands are expressed in different frames");

    static_assert(std::is_same<MiddleFrameOf<Lhs>, RightFrameOf<Rhs>>() or std::is_same<RightFrameOf<Lhs>, MiddleFrameOf<Rhs>>(),
                  "Mismatching frames");

    // clang-format on
};

namespace internal {

// Traits for sum of the form AB + BC
template <typename Lhs, typename Rhs>
struct traits<Sum<Lhs, Rhs>,
              tmp::enable_if_t<std::is_same<RightFrameOf<Lhs>, MiddleFrameOf<Rhs>>{}>>
  : binary_traits_base<Sum<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Lhs>, RightFrameOf<Rhs>>;
};

// Traits for sum with reversed operands BC + AB
template <typename Lhs, typename Rhs>
struct traits<Sum<Lhs, Rhs>,
              tmp::enable_if_t<!std::is_same<RightFrameOf<Lhs>, MiddleFrameOf<Rhs>>{} &&
                               std::is_same<RightFrameOf<Rhs>, MiddleFrameOf<Lhs>>{}>>
  : binary_traits_base<Sum<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Rhs>, MiddleFrameOf<Rhs>, RightFrameOf<Lhs>>;
};

// Traits for invalid sum
// We need this defined so we can at least instantiate the class and fail a static_assert
template <typename Lhs, typename Rhs>
struct traits<Sum<Lhs, Rhs>,
              tmp::enable_if_t<!std::is_same<RightFrameOf<Lhs>, MiddleFrameOf<Rhs>>{} &&
                               !std::is_same<RightFrameOf<Rhs>, MiddleFrameOf<Lhs>>{}>>
  : binary_traits_base<Sum<Lhs, Rhs>> {};

/** Jacobian implementation for all sums */
template <typename Res, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<Sum>, const Res &, const Lhs &, const Rhs &) {
    return identity_t<Res>{};
};

/** Jacobian implementation for all sums */
template <typename Res, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Sum>, const Res &, const Lhs &, const Rhs &) {
    return identity_t<Res>{};
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_SUM_HPP
