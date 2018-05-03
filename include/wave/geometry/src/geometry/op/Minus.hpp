/**
 * @file
 */

#ifndef WAVE_GEOMETRY_NEGATIVE_HPP
#define WAVE_GEOMETRY_NEGATIVE_HPP

namespace wave {

/** Expression representing the negation of a vector expression
 *
 * @tparam Rhs The type of expression being negated
 */
template <typename Rhs>
struct Minus : internal::base_tmpl_t<Rhs, Minus<Rhs>>,
               public UnaryExpression<Minus<Rhs>> {
    // Inherit storage and constructors from UnaryExpression
    using UnaryExpression<Minus<Rhs>>::UnaryExpression;
};

namespace internal {

// Traits for -(framed vector)
template <typename Rhs>
struct traits<Minus<Rhs>, tmp::enable_if_t<has_three_decorators<Rhs>{}>>
  : unary_traits_base<Minus<Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Rhs>, RightFrameOf<Rhs>, MiddleFrameOf<Rhs>>;
};

// Traits for -(scalar)
template <typename Rhs>
struct traits<Minus<Rhs>, tmp::enable_if_t<std::is_arithmetic<eval_t<Rhs>>{}>>
  : unary_traits_base<Minus<Rhs>> {};

/** Jacobian implementation for all Minus */
template <typename Res, typename Rhs>
auto jacobianImpl(expr<Minus>, const Res &, const Rhs &) -> jacobian_t<Res, Rhs> {
    // @todo rewrite in terms of our Identity expression
    return -jacobian_t<Res, Rhs>::Identity();
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_NEGATIVE_HPP
