/**
 * @file
 */

#ifndef WAVE_GEOMETRY_DIVIDE_HPP
#define WAVE_GEOMETRY_DIVIDE_HPP

namespace wave {

/** An expression representing scalar / scalar division
 */
template <typename Lhs, typename Rhs>
struct Divide : internal::base_tmpl_t<Lhs, Rhs, Divide<Lhs, Rhs>>,
                BinaryExpression<Divide<Lhs, Rhs>> {
    // Inherit constructor from BinaryExpression
    using BinaryExpression<Divide<Lhs, Rhs>>::BinaryExpression;
};


namespace internal {

// Traits for left scalar multiplication
// Keep the frames of the vector side.
// @todo add scalar mixing. Currently the type of Lhs is kept.
template <typename Lhs, typename Rhs>
struct traits<Divide<Lhs, Rhs>> : binary_traits_base<Divide<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Lhs>, RightFrameOf<Lhs>>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_DIVIDE_HPP
