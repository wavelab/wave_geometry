/**
 * @file
 * Helpers to identify Eigen and geometric expressions
 */

#ifndef WAVE_GEOMETRY_GEOMETRY_TYPE_TRAITS_HPP
#define WAVE_GEOMETRY_GEOMETRY_TYPE_TRAITS_HPP

namespace wave {
namespace internal {

template <class T>
using is_derived_rotation = std::is_base_of<RotationBase<T>, T>;

template <class T>
using is_derived_rt = std::is_base_of<RigidTransformBase<T>, T>;

template <class T>
using is_derived_transform = std::is_base_of<TransformBase<T>, T>;

template <class T>
using is_derived_twist = std::is_base_of<TwistBase<T>, T>;


// Traits for checking for Eigen expressions

/** Aliases true_type if the T is an Eigen NxN matrix expression */
template <int N, typename T>
using is_eigen_matrix =
  tmp::bool_constant<std::is_base_of<Eigen::MatrixBase<T>, T>::value &&
                     T::RowsAtCompileTime == N && T::ColsAtCompileTime == N>;

/** Aliases true_type if the T is an Eigen quaternion expression */
template <typename T>
struct is_eigen_quaternion : std::is_base_of<Eigen::QuaternionBase<T>, T> {};

/** Evaluates to true_type if the T is an Eigen AngleAxis expression */
template <typename T>
struct is_eigen_angleaxis : std::false_type {};

// AngleAxis is simple; it has no specific base so can be checked directly.
// (Unfortunately this means it can't be Mapped).
template <typename Scalar>
struct is_eigen_angleaxis<Eigen::AngleAxis<Scalar>> : std::true_type {};

/** Aliases true_type if the T is an Eigen Nx1 or 1xN vector expression */
template <int N, typename T>
using is_eigen_vector =
  tmp::bool_constant<T::IsVectorAtCompileTime && T::SizeAtCompileTime == N &&
                     T::IsVectorAtCompileTime>;

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_GEOMETRY_TYPE_TRAITS_HPP
