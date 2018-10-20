/**
 * @file
 */

#ifndef WAVE_GEOMETRY_BOXMINUS_HPP
#define WAVE_GEOMETRY_BOXMINUS_HPP

namespace wave {

/** An expression representing the manifold subtraction of two Lie group elements
 *
 * @f[ SO(3) \times SO(3) \to so(3) @f] or
 * @f[ SE(3) \times SE(3) \to se(3) @f]
 *
 * `BoxMinus<Lhs, Rhs>` is equivalent to `LogMap<RightFrameOf<R>, Compose<Lhs,
 * Inverse<Rhs>>>`.
 */
template <typename Lhs, typename Rhs>
struct BoxMinus
  : internal::base_tmpl_t<typename internal::eval_traits<Lhs>::TangentType,
                          typename internal::eval_traits<Rhs>::TangentType,
                          BoxMinus<Lhs, Rhs>>,
    UnaryExpressionBase<BoxMinus<Lhs, Rhs>, Compose<Lhs, Inverse<Rhs> &&> &&> {
 private:
    using Storage =
      UnaryExpressionBase<BoxMinus<Lhs, Rhs>, Compose<Lhs, Inverse<Rhs> &&> &&>;

 public:
    // Inherit constructors from BinaryExpression
    using Storage::Storage;

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(), "Frame mismatch");
    static_assert(std::is_same<RightFrameOf<Lhs>, RightFrameOf<Rhs>>(), "Frame mismatch");
};

namespace internal {

/** We implement BoxMinus as a "heavy alias": it is meant to act exactly like the LogMap
 * expression. See also BoxPlus.
 */
template <typename Lhs, typename Rhs>
struct traits<BoxMinus<Lhs, Rhs>>
  : traits<LogMap<RightFrameOf<Rhs>, Compose<Lhs, Inverse<Rhs> &&> &&>> {};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_BOXMINUS_HPP
