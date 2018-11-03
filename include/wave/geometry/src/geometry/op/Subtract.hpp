/**
 * @file
 */

#ifndef WAVE_GEOMETRY_SUBTRACT_HPP
#define WAVE_GEOMETRY_SUBTRACT_HPP

namespace wave {

/** An expression representing the difference two affine or vector expressions.
 *
 * For example, this expression can be applied to two Translations, or to a Point and a
 * Translation. The subtrahend (rhs) must be the tangent type of the minuend (lhs).
 */
template <typename Lhs, typename Rhs>
struct Subtract : internal::subtract_base_tmpl_t<Lhs, Rhs>,
                  internal::binary_storage_for<Subtract<Lhs, Rhs>> {
    static_assert(
      internal::same_base_tmpl<typename internal::eval_traits<Lhs>::TangentType,
                               typename internal::eval_traits<Rhs>::TangentType>{},
      "Operands to Subtract have mismatching difference types");

    // Inherit constructor from BinaryStorage
    using Storage = internal::binary_storage_for<Subtract<Lhs, Rhs>>;
    using Storage::Storage;

    // clang-format off
    // keep static_assert on one line for compiler output

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Operands are expressed in different frames");

    static_assert(std::is_same<RightFrameOf<Lhs>, RightFrameOf<Rhs>>() or std::is_same<MiddleFrameOf<Lhs>, MiddleFrameOf<Rhs>>(),
                  "Mismatching frames");

    // clang-format on
};

namespace internal {

// Chooses the conceptual base for Subtract.
// Case for point - vector: choose point's base
template <typename Lhs, typename Rhs>
struct subtract_base_tmpl<
  Lhs,
  Rhs,
  std::enable_if_t<same_base_tmpl<typename eval_traits<Lhs>::TangentType, Rhs>{} &&
                   !same_base_tmpl<Lhs, typename eval_traits<Rhs>::TangentType>{}>> {
    using type = base_tmpl_t<Lhs, Subtract<Lhs, Rhs>>;
};

// Case for vector - vector: choose vectors' base (which must match)
template <typename Lhs, typename Rhs>
struct subtract_base_tmpl<
  Lhs,
  Rhs,
  std::enable_if_t<same_base_tmpl<typename eval_traits<Lhs>::TangentType, Rhs>{} &&
                   same_base_tmpl<Lhs, typename eval_traits<Rhs>::TangentType>{}>> {
    using type = base_tmpl_t<Lhs, Rhs, Subtract<Lhs, Rhs>>;
};

// Case for point - point: choose tangent types' base (which must match)
template <typename Lhs, typename Rhs>
struct subtract_base_tmpl<
  Lhs,
  Rhs,
  std::enable_if_t<!same_base_tmpl<typename eval_traits<Lhs>::TangentType, Rhs>{} &&
                   !same_base_tmpl<Lhs, typename eval_traits<Rhs>::TangentType>{}>> {
    using type = base_tmpl_t<typename eval_traits<Lhs>::TangentType,
                             typename eval_traits<Rhs>::TangentType,
                             Subtract<Lhs, Rhs>>;
};

// Traits for Subtract of the form AB - CB = AC
template <typename Lhs, typename Rhs>

struct traits<Subtract<Lhs, Rhs>,
              std::enable_if_t<std::is_same<RightFrameOf<Lhs>, RightFrameOf<Rhs>>{}>>
    : binary_traits_base<Subtract<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Lhs>, MiddleFrameOf<Rhs>>;
};

// Traits for Subtract of the form BC - BA = AC
template <typename Lhs, typename Rhs>
struct traits<Subtract<Lhs, Rhs>,
              std::enable_if_t<std::is_same<MiddleFrameOf<Lhs>, MiddleFrameOf<Rhs>>{} &&
                               !std::is_same<RightFrameOf<Lhs>, RightFrameOf<Rhs>>{}>>
    : binary_traits_base<Subtract<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, RightFrameOf<Rhs>, RightFrameOf<Lhs>>;
};

// Traits for invalid Subtract
// We need this defined so we can at least instantiate the class and fail a static_assert
template <typename Lhs, typename Rhs>
struct traits<Subtract<Lhs, Rhs>,
              std::enable_if_t<!std::is_same<MiddleFrameOf<Lhs>, MiddleFrameOf<Rhs>>{} &&
                               !std::is_same<RightFrameOf<Lhs>, RightFrameOf<Rhs>>{}>>
    : binary_traits_base<Subtract<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Lhs>, MiddleFrameOf<Rhs>>;
};

/** Jacobian implementation for all subtractions */
template <typename Res, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<Subtract>, const Res &, const Lhs &, const Rhs &) {
    return identity_t<Lhs>{};
};

/** Jacobian implementation for all subtractions */
template <typename Res, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<Subtract>, const Res &, const Lhs &, const Rhs &) {
    // @todo rewrite in terms of our Identity expression
    return -jacobian_t<Res, Rhs>::Identity();
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_SUBTRACT_HPP
