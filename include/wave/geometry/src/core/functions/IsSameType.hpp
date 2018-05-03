/**
 * @file
 */

#ifndef WAVE_GEOMETRY_ISSAMETYPE_HPP
#define WAVE_GEOMETRY_ISSAMETYPE_HPP

namespace wave {
namespace internal {

/** Determines whether `a` and `b` are the same type. If so, asserts that they are also
 * the same object *in debug mode only*.
 *
 * @warning experimental
 */
// Specialization when types are not the same
template <typename A, typename B>
inline constexpr bool isSameType(const ExpressionBase<A> &,
                                 const ExpressionBase<B> &) noexcept {
    return false;
}
#ifndef NDEBUG
// Specialization when types are the same, with debug-mode assert that identities are same
template <typename A>
inline constexpr bool isSameType(const ExpressionBase<A> &a,
                                 const ExpressionBase<A> &b) noexcept {
    // Hacky way of asserting in constexpr fn. Throw will terminate since it's noexcept.
    // See http://ericniebler.com/2014/09/27/assert-and-constexpr-in-cxx11/
    return isSame(a, b) ? true : throw std::logic_error("Assert failed");
}
#else
// Specialization when types are the same
template <typename A>
inline constexpr bool isSameType(const ExpressionBase<A> &,
                                 const ExpressionBase<A> &) noexcept {
    return true;
}
#endif


/** Determines whether `a` contains an expression of the type of `b`.
 *
 * @warning experimental
 */
template <typename A, typename B, typename Enable = void>
struct contains_same_type;

template <typename A, typename B>
struct contains_same_type<A, B, enable_if_leaf_t<A>> : std::is_same<A, B> {};

template <typename A, typename B>
struct contains_same_type<A, B, enable_if_unary_t<A>>
  : tmp::bool_constant<std::is_same<A, B>{} ||
                       contains_same_type<typename A::RhsDerived, B>{}> {};

template <typename A, typename B>
struct contains_same_type<A, B, enable_if_binary_t<A>>
  : tmp::bool_constant<std::is_same<A, B>{} ||
                       contains_same_type<typename A::LhsDerived, B>{} ||
                       contains_same_type<typename A::RhsDerived, B>{}> {};


}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_ISSAMETYPE_HPP
