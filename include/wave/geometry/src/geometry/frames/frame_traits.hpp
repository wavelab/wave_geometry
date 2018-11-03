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

// Static assert whether frames match, in a way that shows the frames in compiler message
// Where we have an expression Rhs, we use this by instantiating with plain_output_t<Rhs>
template <typename Lhs, typename Rhs>
inline void assert_same_frames() {
    static_assert(same_frames<Lhs, Rhs>{}, "Mismatching frames");
};

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
struct WrapWithFrames;

// Case for two frames
template <typename FL, typename FR>
struct WrapWithFrames<FL, FR> {
    template <typename Arg>
    auto operator()(Arg &&leaf) const -> Framed<tmp::remove_cr_t<Arg>, FL, FR> {
        using FramedType = Framed<tmp::remove_cr_t<Arg>, FL, FR>;
        return FramedType{typename FramedType::from_unframed{}, std::forward<Arg>(leaf)};
    }
};

// Case for three frames
template <typename FL, typename FM, typename FR>
struct WrapWithFrames<FL, FM, FR> {
    template <typename Arg,
              typename std::enable_if_t<has_three_decorators<Arg>{}, bool> = true>
    auto operator()(Arg &&leaf) const -> Framed<tmp::remove_cr_t<Arg>, FL, FM, FR> {
        using FramedType = Framed<tmp::remove_cr_t<Arg>, FL, FM, FR>;
        return FramedType{typename FramedType::from_unframed{}, std::forward<Arg>(leaf)};
    }

    // Wrap a point<A, B> with frames given as A, A, B
    // Simplifies the implementation of points by reusing some vector traits
    // @todo cleanup? See also middle_or_left_frame_impl.
    template <
      typename Arg,
      typename std::enable_if_t<has_two_decorators<Arg>{} && std::is_same<FL, FM>{},
                                bool> = true>
    auto operator()(Arg &&leaf) const -> Framed<tmp::remove_cr_t<Arg>, FL, FR> {
        using FramedType = Framed<tmp::remove_cr_t<Arg>, FL, FR>;
        return FramedType{typename FramedType::from_unframed{}, std::forward<Arg>(leaf)};
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
    using PreparedType = const Derived &;
    using UniqueLeaves = has_unique_leaves_leaf<Derived>;

    // In evaluation we strip the frames
    using Tag = expr<Framed>;
    using EvalType = eval_t<WrappedLeaf>;

    // Before user-facing output we add the frames back
    using OutputFunctor = WrapWithFrames<Frames...>;
};

// Get either the middle or the left frame for MiddleFrameOf
// This slight hack allows points to have a MiddleFrameOf trait. Points declared as
// A_p_B (e.g. PointFd<A, B>) are internally treated as A_p_AB.
// @todo clean up?
template <typename Derived, typename = void>
struct middle_or_left_frame_impl;

template <typename Derived>
struct middle_or_left_frame_impl<Derived,
                                 std::enable_if_t<has_three_decorators<Derived>{}>> {
    using type = typename internal::eval_traits<Derived>::MiddleFrame;
};

template <typename Derived>
struct middle_or_left_frame_impl<Derived,
                                 std::enable_if_t<!has_three_decorators<Derived>{}>> {
    using type = typename internal::eval_traits<Derived>::LeftFrame;
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
  typename internal::middle_or_left_frame_impl<tmp::remove_cr_t<Derived>>::type;

/** Gets an expression's right frame descriptor */
template <typename Derived>
using RightFrameOf =
  typename internal::eval_traits<tmp::remove_cr_t<Derived>>::RightFrame;

}  // namespace wave

#endif  // WAVE_GEOMETRY_FRAME_TRAITS_HPP
