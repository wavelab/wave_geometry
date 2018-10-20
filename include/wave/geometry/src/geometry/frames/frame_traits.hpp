/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_FRAME_TRAITS_HPP
#define WAVE_GEOMETRY_FRAME_TRAITS_HPP

namespace wave {
namespace internal {

/** Convenience traits base for a leaf expression with two decorators */
struct frameable_transform_traits {
    using LeftFrame = NoFrame;
    using RightFrame = NoFrame;
};

/** Convenience traits base for a leaf expression with three decorators */
struct frameable_vector_traits {
    using LeftFrame = NoFrame;
    using MiddleFrame = NoFrame;
    using RightFrame = NoFrame;
};


// Traits of framed types

TICK_TRAIT(has_three_decorators) {
    template <class D>
    auto require(D &&)
      ->valid<typename eval_traits<D>::LeftFrame,
              typename eval_traits<D>::MiddleFrame,
              typename eval_traits<D>::RightFrame>;
};

TICK_TRAIT(has_two_decorators) {
    template <class D>
    auto require(D &&)
      ->valid<typename eval_traits<D>::LeftFrame,
              typename eval_traits<D>::RightFrame,
              is_false<has_three_decorators<D>>>;
};

/** Check if the expression has exactly matching frames, via traits */
template <typename LhsDerived, typename RhsDerived, typename Enable = void>
struct same_frames : std::false_type {};

template <typename Lhs, typename Rhs>
struct same_frames<
  Lhs,
  Rhs,
  std::enable_if_t<has_three_decorators<Lhs>{} && has_three_decorators<Rhs>{}>>
  : std::integral_constant<bool,
                           std::is_same<typename eval_traits<Lhs>::LeftFrame,
                                        typename eval_traits<Rhs>::LeftFrame>{} &&
                             std::is_same<typename eval_traits<Lhs>::MiddleFrame,
                                          typename eval_traits<Rhs>::MiddleFrame>{} &&
                             std::is_same<typename eval_traits<Lhs>::RightFrame,
                                          typename eval_traits<Rhs>::RightFrame>{}> {};

template <typename Lhs, typename Rhs>
struct same_frames<
  Lhs,
  Rhs,
  std::enable_if_t<has_two_decorators<Lhs>{} && has_two_decorators<Rhs>{}>>
  : std::integral_constant<bool,
                           std::is_same<typename eval_traits<Lhs>::LeftFrame,
                                        typename eval_traits<Rhs>::LeftFrame>{} &&
                             std::is_same<typename eval_traits<Lhs>::RightFrame,
                                          typename eval_traits<Rhs>::RightFrame>{}> {};


template <typename...>
struct is_unframed_impl : std::false_type {};

template <>
struct is_unframed_impl<NoFrame, NoFrame> : std::true_type {};

template <>
struct is_unframed_impl<NoFrame, NoFrame, NoFrame> : std::true_type {};

/** Check if the expression is unframed: either it has no frame traits, or it has NoFrame
 * set for all descriptors.
 *
 * @note even an explicitly defined type Framed<..., NoFrame, NoFrame> will evaluate true.
 */
template <typename Derived, typename Enable = void>
struct is_unframed : std::false_type {};

template <typename Derived>
struct is_unframed<Derived, std::enable_if_t<has_two_decorators<Derived>{}>>
  : is_unframed_impl<typename eval_traits<Derived>::LeftFrame,
                     typename eval_traits<Derived>::RightFrame> {};

template <typename Derived>
struct is_unframed<Derived, std::enable_if_t<has_three_decorators<Derived>{}>>
  : is_unframed_impl<typename eval_traits<Derived>::LeftFrame,
                     typename eval_traits<Derived>::MiddleFrame,
                     typename eval_traits<Derived>::RightFrame> {};


template <typename... Frames>
struct add_frames {
    template <typename Derived>
    using to = Framed<Derived, Frames...>;
};

template <>
struct add_frames<NoFrame, NoFrame> {
    template <typename Derived>
    using to = Derived;
};

template <>
struct add_frames<NoFrame, NoFrame, NoFrame> {
    template <typename Derived>
    using to = Derived;
};

/** Output functor for use by framed expressions */
template <typename... Frames>
struct WrapWithFrames {
    template <typename Arg>
    auto operator()(Arg &&leaf) const -> Framed<tmp::remove_cr_t<Arg>, Frames...> {
        return Framed<tmp::remove_cr_t<Arg>, Frames...>{std::forward<Arg>(leaf)};
    }
};

/** When something has NoFrame traits we don't wrap it */
template <>
struct WrapWithFrames<NoFrame, NoFrame> : IdentityFunctor {};

/** When something has NoFrame traits we don't wrap it */
template <>
struct WrapWithFrames<NoFrame, NoFrame, NoFrame> : IdentityFunctor {};

// Base traits for frames
template <typename WrappedLeaf, typename... Frames>
struct frames_traits_base : traits<WrappedLeaf> {
 private:
    using Derived = Framed<WrappedLeaf, Frames...>;

 public:
    // Copy most traits from the wrapped Leaf, but override some
    using PreparedType = Derived &;
    using UniqueLeaves = has_unique_leaves_leaf<Derived>;

    // In evaluation we strip the frames
    using Tag = expr<Framed>;
    using EvalType = eval_t<WrappedLeaf>;

    // Before user-facing output we add the frames back
    using OutputFunctor = WrapWithFrames<Frames...>;
};

}  // namespace internal

// These aliases don't follow the usual naming convention and are in the main namespace
// because they are meant to be part of readable error messages

/** Gets an expression's left frame descriptor */
template <typename Derived>
using LeftFrameOf = typename internal::eval_traits<tmp::remove_cr_t<Derived>>::LeftFrame;

/** Gets an expression's middle frame descriptor */
template <typename Derived>
using MiddleFrameOf =
  typename internal::eval_traits<tmp::remove_cr_t<Derived>>::MiddleFrame;

/** Gets an expression's right frame descriptor */
template <typename Derived>
using RightFrameOf =
  typename internal::eval_traits<tmp::remove_cr_t<Derived>>::RightFrame;

}  // namespace wave

#endif  // WAVE_GEOMETRY_FRAME_TRAITS_HPP
