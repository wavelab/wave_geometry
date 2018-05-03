/**
 * @file
 * Helpers to identify Eigen and geometric expressions
 */

#ifndef WAVE_GEOMETRY_GEOMETRY_TYPE_TRAITS_HPP
#define WAVE_GEOMETRY_GEOMETRY_TYPE_TRAITS_HPP

namespace wave {
namespace internal {

TICK_TRAIT(valid_vector_traits, valid_expression_traits<_>) {
    template <class T>
    auto require(T &&)
      ->valid<has_template<T::template rebind>,
              has_type<typename T::ImplType,
                       std::is_base_of<Eigen::MatrixBase<typename T::ImplType>, _>>,
              is_true_c<T::Size == T::TangentSize>>;
};

TICK_TRAIT(has_valid_vector_traits) {
    template <class T>
    auto require(T &&)
      ->valid<is_true<valid_vector_traits<typename ::wave::internal::traits<T>>>>;
};


TICK_TRAIT(is_vector_leaf, is_leaf_expression<_>, has_valid_vector_traits<_>) {
    template <class T>
    auto require(T && x)
      ->valid<is_true<std::is_base_of<VectorBase<T>, T>>,
              std::is_same<typename internal::traits<T>::ImplType,
                           tmp::remove_cr_t<decltype(x.value())>>>;
};

template <class T>
using is_derived_rotation = std::is_base_of<RotationBase<T>, T>;

template <class T>
using is_derived_rt = std::is_base_of<RigidTransformBase<T>, T>;

template <class T>
using is_derived_transform = std::is_base_of<TransformBase<T>, T>;

template <class T>
using is_derived_twist = std::is_base_of<TwistBase<T>, T>;

TICK_TRAIT(is_rt_leaf, is_leaf_expression<_>) {
    template <class T>
    auto require(T && x)
      ->valid<is_true<is_derived_rt<T>>,
              decltype(x.rotation()),
              decltype(x.translation())>;
};

TICK_TRAIT(is_transform_leaf, is_leaf_expression<_>) {
    template <class T>
    auto require(T && x)
      ->valid<is_true<is_derived_transform<T>>,
              decltype(x.rotation()),
              decltype(x.translation())>;
};

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
