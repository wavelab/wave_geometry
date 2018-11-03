/**
 * @file
 */

#ifndef WAVE_GEOMETRY_GEOMETRY_FORWARD_DECLARATIONS_HPP
#define WAVE_GEOMETRY_GEOMETRY_FORWARD_DECLARATIONS_HPP


namespace wave {
// Forward declarations

template <typename WrappedLeaf, typename... Frames>
class Framed;

struct NoFrame;

template <typename Derived>
struct CompoundBase;

template <typename Derived>
struct AffineBase;

template <typename Derived>
struct VectorBase;

template <typename Derived>
struct ScalarBase;

template <typename ToDerived, typename FromDerived>
struct Conversion;

template <typename Lhs, typename Rhs>
struct Sum;

template <typename Lhs, typename Rhs>
struct Subtract;

template <typename Rhs>
struct Minus;

template <typename Lhs, typename Rhs>
struct Scale;

template <typename Lhs, typename Rhs>
struct ScaleR;

template <typename Lhs, typename Rhs>
struct ScaleDiv;

template <typename Lhs, typename Rhs>
struct Product;

template <typename Lhs, typename Rhs>
struct Divide;

template <typename Lhs, typename Rhs>
struct Rotate;

template <typename Lhs, typename Rhs>
struct Transform;

template <typename Lhs, typename Rhs>
struct Compose;

template <typename Lhs, typename Rhs>
struct ComposeFlipped;

template <typename Rhs>
struct Inverse;

template <typename Rhs>
struct ExpMap;

template <typename Rhs, typename ExtraFrame>
struct LogMap;

template <typename Lhs, typename Rhs>
struct BoxPlus;

template <typename Lhs, typename Rhs>
struct BoxMinus;

template <typename Lhs, typename Rhs>
struct CompoundBoxPlus;

template <typename Lhs, typename Rhs>
struct CompoundBoxMinus;

template <typename ImplType>
class MatrixRotation;

template <typename ImplType>
class QuaternionRotation;

template <typename ImplType>
class AngleAxisRotation;

template <typename Derived>
class RotationBase;

template <typename ImplTypee>
class RelativeRotation;

template <typename Derived>
struct RelativeRotationBase;

template <typename ImplType>
class Twist;

template <typename Derived>
struct TwistBase;

template <typename Derived>
struct TranslationBase;

template <typename ImplType>
class Translation;

template <typename Derived>
class TransformBase;

template <typename Derived>
class RigidTransformBase;

template <typename Derived>
class MatrixRigidTransform;

template <typename QuatType, typename VecType>
class CompactRigidTransform;

template <typename Leaf>
class Zero;

template <typename Leaf>
class Identity;

template <typename ScalarType>
class Scalar;

namespace internal {

// Chooses the conceptual base for Sum
template <typename Lhs, typename Rhs, typename = void>
struct sum_base_tmpl;
template <typename Lhs, typename Rhs>
using sum_base_tmpl_t = typename sum_base_tmpl<Lhs, Rhs>::type;

// Chooses the conceptual base for Subtract
template <typename Lhs, typename Rhs, typename = void>
struct subtract_base_tmpl;
template <typename Lhs, typename Rhs>
using subtract_base_tmpl_t = typename subtract_base_tmpl<Lhs, Rhs>::type;

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_GEOMETRY_FORWARD_DECLARATIONS_HPP
