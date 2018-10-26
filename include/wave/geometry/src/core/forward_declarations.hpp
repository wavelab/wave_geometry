/**
 * @file
 */

#ifndef WAVE_GEOMETRY_FORWARD_DECLARATIONS_HPP
#define WAVE_GEOMETRY_FORWARD_DECLARATIONS_HPP

namespace wave {

namespace internal {

template <typename T, typename Enable = void>
struct traits;

// There are no distinct traits of a const type
template <typename T>
struct traits<const T> : traits<T> {};

// There are no distinct traits of a reference type
template <typename T>
struct traits<T &> : traits<T> {};

// There are no distinct traits of a reference type
template <typename T>
struct traits<T &&> : traits<T> {};

/** Marker of a missing expression implemenation */
struct NotImplemented;

/** Marker of a disabled call (should be accompanied by static_assert) */
struct NotAllowed {};

/** Marker of ADL-enabled functions in internal namespace */
struct adl {};

}  // namespace internal

template <typename Derived, typename StorageType>
struct LeafStorage;

template <typename Derived, typename RhsDerived>
struct UnaryStorage;

template <typename Derived, typename LhsDerived, typename RhsDerived>
struct BinaryStorage;

template <typename Derived, typename... Primitives>
struct NaryStorage;

template <typename Derived>
class ExpressionBase;

template <typename ToDerived, typename FromDerived>
struct Convert;

template <typename Fn, typename RhsDerived>
struct MemberAccess;

}  // namespace wave

#endif  // WAVE_GEOMETRY_FORWARD_DECLARATIONS_HPP
