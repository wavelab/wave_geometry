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
struct BoxMinus : internal::base_tmpl_t<typename internal::eval_traits<Lhs>::TangentType,
                                        typename internal::eval_traits<Rhs>::TangentType,
                                        BoxMinus<Lhs, Rhs>>,
                  UnaryStorage<BoxMinus<Lhs, Rhs>, Compose<Lhs, Inverse<Rhs>>> {
 private:
    using Storage = UnaryStorage<BoxMinus<Lhs, Rhs>, Compose<Lhs, Inverse<Rhs>>>;

 public:
    // Inherit constructors from BinaryStorage
    using Storage::Storage;

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(), "Frame mismatch");
    static_assert(std::is_same<RightFrameOf<Lhs>, RightFrameOf<Rhs>>(), "Frame mismatch");
};


/** An expression representing the manifold subtraction of two compound +-manifold
 * elements
 *
 * @f[ S \times S \to R^n @f]
 */
template <typename Lhs, typename Rhs>
struct CompoundBoxMinus
    : internal::base_tmpl_t<typename internal::eval_traits<Lhs>::TangentType,
                            typename internal::eval_traits<Rhs>::TangentType,
                            CompoundBoxMinus<Lhs, Rhs>>,
      internal::binary_storage_for<CompoundBoxMinus<Lhs, Rhs>> {
 private:
    using Storage = internal::binary_storage_for<CompoundBoxMinus<Lhs, Rhs>>;

 public:
    // Inherit constructors from BinaryStorage
    using Storage::Storage;
};


namespace internal {

/** We implement BoxMinus as a "heavy alias": it is meant to act exactly like the LogMap
 * expression. See also BoxPlus.
 */
template <typename Lhs, typename Rhs>
struct traits<BoxMinus<Lhs, Rhs>>
    : traits<LogMap<RightFrameOf<Rhs>, Compose<Lhs, Inverse<Rhs>>>> {};


template <typename Lhs, typename Rhs>
struct traits<CompoundBoxMinus<Lhs, Rhs>>
    : binary_traits_base<CompoundBoxMinus<Lhs, Rhs>> {};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_BOXMINUS_HPP
