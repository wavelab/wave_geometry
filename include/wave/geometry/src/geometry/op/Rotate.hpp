/**
 * @file
 */

#ifndef WAVE_GEOMETRY_ROTATE_HPP
#define WAVE_GEOMETRY_ROTATE_HPP

namespace wave {

/** A translation expression representing the product of a Rotation and a Translation
 *
 * @tparam Lhs a rotation expression in SO(3)
 * @tparam Rhs a translation in R^3
 */
template <typename Lhs, typename Rhs>
struct Rotate : public internal::base_tmpl_t<Rhs, Rotate<Lhs, Rhs>>,
                public BinaryExpression<Rotate<Lhs, Rhs>> {
    // Inherit constructors from BinaryExpression
    using BinaryExpression<Rotate<Lhs, Rhs>>::BinaryExpression;

    static_assert(std::is_same<RightFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Mismatching frames");
};

namespace internal {

template <typename Lhs, typename Rhs>
struct traits<Rotate<Lhs, Rhs>> : binary_traits_base<Rotate<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Rhs>, RightFrameOf<Rhs>>;
};

/** Jacobian of any rotation wrt to the lhs */
template <typename Val, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<Rotate>,
                      const Translation<Val> &val,
                      const RotationBase<Lhs> &,
                      const TranslationBase<Rhs> &)
  -> decltype(crossMatrix(-val.value())) {
    // Bloesch equation 68
    return crossMatrix(-val.value());
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_ROTATE_HPP
