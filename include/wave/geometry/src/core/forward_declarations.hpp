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

// There are no distinct traits of a && type
template <typename T>
struct traits<T &&> : traits<T> {};

/** Marker of a missing expression implemenation */
struct NotImplemented;

/** Marker of a disabled call (should be accompanied by static_assert) */
struct NotAllowed {};

/** Every expression must have an ExpressionImpl with these members:
 * @todo */
template <typename Derived, typename Enable = void>
struct ExpressionImpl;

}  // namespace internal

template <typename StorageType, typename Derived>
struct LeafExpression;

template <typename Derived>
struct UnaryExpression;

template <typename Derived>
struct BinaryExpression;

template <typename Derived>
class ExpressionBase;

template <typename ToDerived, typename FromDerived>
struct Convert;

}  // namespace wave

#endif  // WAVE_GEOMETRY_FORWARD_DECLARATIONS_HPP
