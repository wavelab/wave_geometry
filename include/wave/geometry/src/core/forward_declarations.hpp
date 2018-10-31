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

/** Check if the expression has exactly matching frames, via traits */
template <typename Lhs, typename Rhs, typename Enable = void>
struct same_frames;

/** Check if the expression has no frames */
template <typename Derived, typename Enable = void>
struct is_unframed : std::true_type {};

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

template <typename L, typename R>
class BlockMatrix;

}  // namespace wave

#endif  // WAVE_GEOMETRY_FORWARD_DECLARATIONS_HPP
