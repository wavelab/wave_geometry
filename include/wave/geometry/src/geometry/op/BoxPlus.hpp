/**
 * @file
 */

#ifndef WAVE_GEOMETRY_BOXPLUS_HPP
#define WAVE_GEOMETRY_BOXPLUS_HPP

namespace wave {

/** An expression representing the manifold addition of a Lie group element and a Lie
 * algebra element.
 *
 * @f[ SO(3) \times so(3) \to SO(3) @f] or
 * @f[ SE(3) \times se(3) \to SE(3) @f]
 *
 * `BoxPlus<Lhs, Rhs>` is equivalent to `Compose<ExpMap<Rhs>, Lhs>` (note the switched
 * operands), except Lhs is evaluated first. It is provided for readability and
 * predictability of evaluation order. Specifically, the result of
 *
 *      (BoxPlus<Lhs, Rhs>{...}).evalWithJacobians()
 *
 * follows the natural reading order: tuple<Value, JacWrtLhs, JacWrtRhs>
 * (The above type names are descriptive but not real).
 *
 * @note BoxPlus is implemented as a wrapper for `ComposeFlipped<Lhs, ExpMap<Rhs>>` (see
 * `internal::traits<BoxPlus<Lhs, Rhs>>` for details).
 */
template <typename Lhs, typename Rhs>
struct BoxPlus : internal::base_tmpl_t<Lhs, BoxPlus<Lhs, Rhs>>,
                 BinaryExpression<ComposeFlipped<Lhs, ExpMap<Rhs>>> {
 private:
    using Storage = BinaryExpression<ComposeFlipped<Lhs, ExpMap<Rhs>>>;

 public:
    using Storage::Storage;

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(), "Frame mismatch");
    static_assert(std::is_same<LeftFrameOf<Lhs>, MiddleFrameOf<Rhs>>(), "Frame mismatch");
    static_assert(std::is_same<RightFrameOf<Lhs>, RightFrameOf<Rhs>>(), "Frame mismatch");
};

namespace internal {

/** We implement BoxPlus as a "heavy alias": it is meant to act exactly like
 * `ComposeFlipped<Lhs, ExpMap<Rhs>>`, but hide this detail from the user.
 *
 * This setup has some advantages over the alternatives:
 *
 *   - If `operator+(Lhs, Rhs)` returned `Compose<ExpMap<Rhs>, Lhs>`, Jacobians would be
 *   returned in an unexpected order (Rhs, Lhs), though the values are equivalent.
 *   - If `operator+(Lhs, Rhs)` returned something like `ComposeFlipped<Lhs,
 *   ExpMap<Rhs>>`, it would fix the order problem. However, it puts a hard-to-read type
 *   in the user-facing expression.
 *   - If we had a `BoxPlus` expression and simply changed the `evalImpl()` to call the
 *   `evalImpl()` of `Compose` and `ExpMap`, `BoxPlus` would not be equivalent to
 *   `Compose<ExpMap<Rhs>, Lhs>` because the Prepare step would not be able to add
 *   conversions between ExpMap and Compose. The Jacobian evaluator would also not be able
 *   to use the intermediate result of ExpMap.
 *
 *  The traits are therefore identical to `ComposeFlipped<Lhs, ExpMap<Rhs>>`.
 */
template <typename Lhs, typename Rhs>
struct traits<BoxPlus<Lhs, Rhs>> : traits<ComposeFlipped<Lhs, ExpMap<Rhs>>> {};


}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_BOXPLUS_HPP
