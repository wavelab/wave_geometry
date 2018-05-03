/**
 * @file
 */

#ifndef WAVE_GEOMETRY_ISSAME_HPP
#define WAVE_GEOMETRY_ISSAME_HPP

namespace wave {

// Not in internal namespace due to some ADL(?) problem with recursive function templates
// @todo figure this out and move to internal namespace

/** Attempt to determine whether `a` and `b` are identical expressions.
 *
 * The criteria are
 *  - `a` and `b` have the same derived type
 *  - leaf expressions in `a` and `b` have the same storage address
 *
 *  @warning This checks for identity in terms of C++ objects, not mathematical equality
 *  or equality of contents. Expressions which evaluate to the same result may not be
 *  considered identical by isSame.
 */
template <typename A, typename B>
inline constexpr bool isSame(const ExpressionBase<A> &,
                             const ExpressionBase<B> &) noexcept {
    static_assert(!std::is_same<A, B>::value, "Should only be called when types differ");
    return false;
}

template <typename Derived, internal::enable_if_leaf_t<Derived, int> = 0>
inline constexpr bool isSame(const ExpressionBase<Derived> &a,
                             const ExpressionBase<Derived> &b) noexcept {
    return &a.derived() == &b.derived();
}


template <typename Derived, internal::enable_if_unary_t<Derived, int> = 0>
inline constexpr bool isSame(const ExpressionBase<Derived> &a,
                             const ExpressionBase<Derived> &b) noexcept {
    return isSame(a.derived().rhs(), b.derived().rhs());
}

template <typename Derived, internal::enable_if_binary_t<Derived, int> = 0>
inline constexpr bool isSame(const ExpressionBase<Derived> &a,
                             const ExpressionBase<Derived> &b) noexcept {
    return isSame(a.derived().lhs(), b.derived().lhs()) &&
           isSame(a.derived().rhs(), b.derived().rhs());
}

/** Attempt to determine whether `a` contains an expression identical to `b`
 *
 * @see isSame
 *
 *  @warning This checks for identity in terms of C++ objects, not mathematical equality
 *  or equality of contents. Expressions which evaluate to the same result may not be
 *  considered identical by isSame.
 */
template <typename A>
inline constexpr bool containsSame(const ExpressionBase<A> &a,
                                   const ExpressionBase<A> &b) noexcept {
    return &a.derived().value() == &b.derived().value();
}

template <typename A, typename B, internal::enable_if_leaf_t<A, int> = 0>
inline constexpr bool containsSame(const ExpressionBase<A> &,
                                   const ExpressionBase<B> &) noexcept {
    static_assert(!std::is_same<A, B>{}, "Should only be called when types differ");
    return false;
}

template <typename A, typename B, internal::enable_if_unary_t<A, int> = 0>
inline constexpr bool containsSame(const ExpressionBase<A> &a,
                                   const ExpressionBase<B> &b) noexcept {
    return std::is_same<A, B>{} || containsSame(a.derived().rhs(), b);
}

template <typename A, typename B, internal::enable_if_binary_t<A, int> = 0>
inline constexpr bool containsSame(const ExpressionBase<A> &a,
                                   const ExpressionBase<B> &b) noexcept {
    return std::is_same<A, B>{} || containsSame(a.derived().lhs(), b) ||
           containsSame(a.derived().rhs(), b);
}

}  // namespace wave

#endif  // WAVE_GEOMETRY_ISSAME_HPP
