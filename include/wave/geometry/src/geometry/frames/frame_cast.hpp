/**
 * @file
 */

#ifndef WAVE_GEOMETRY_FRAME_CAST_HPP
#define WAVE_GEOMETRY_FRAME_CAST_HPP

namespace wave {

/** Represents the result of frame_cast<>()
 *
 * Unlike Framed, it does not have its own storage, as is not meant to be used directly.
 *
 * Unlike Framed, the rhs type comes last in the template parameters. This way, when
 * printed by the compiler, the new frames will be on the left of the (possibly long) rhs
 * expression, matching the Convert expression and the call to frame_cast.
 */
template <typename...>
struct FrameCast;

template <typename F1, typename F2, typename Rhs>
struct FrameCast<F1, F2, Rhs> : internal::base_tmpl_t<Rhs, FrameCast<F1, F2, Rhs>>,
                                UnaryStorage<FrameCast<F1, F2, Rhs>, Rhs> {
 private:
    using Storage = UnaryStorage<FrameCast<F1, F2, Rhs>, Rhs>;

 public:
    using Storage::Storage;
};

template <typename F1, typename F2, typename F3, typename Rhs>
struct FrameCast<F1, F2, F3, Rhs>
    : internal::base_tmpl_t<Rhs, FrameCast<F1, F2, F3, Rhs>>,
      UnaryStorage<FrameCast<F1, F2, F3, Rhs>, Rhs> {
 private:
    using Storage = UnaryStorage<FrameCast<F1, F2, F3, Rhs>, Rhs>;

 public:
    using Storage::Storage;
};


/**
 * Forcibly changes the frame descriptors of an existing vector expression.
 *
 * @tparam F1, F2, F3 the new frame descriptors
 */
template <typename F1, typename F2, typename F3, typename Rhs>
auto frame_cast(const ExpressionBase<Rhs> &rhs) -> TICK_FUNCTION_REQUIRES(
  internal::has_three_decorators<Rhs>{})(FrameCast<F1, F2, F3, Rhs>) {
    return FrameCast<F1, F2, F3, Rhs>{rhs.derived()};
}

/**
 * Forcibly changes the frame descriptors of an existing vector expression.
 *
 * @tparam F1, F2, F3 the new frame descriptors
 */
template <typename F1, typename F2, typename F3, typename Rhs>
auto frame_cast(ExpressionBase<Rhs> &&rhs) -> TICK_FUNCTION_REQUIRES(
  internal::has_three_decorators<Rhs>{})(FrameCast<F1, F2, F3, Rhs &&>) {
    return FrameCast<F1, F2, F3, Rhs &&>{std::move(rhs.derived())};
}

/**
 * Forcibly changes the frame descriptors of an existing transform expression.
 *
 * @tparam F1, F2 the new frame descriptors
 */
template <typename F1, typename F2, typename Rhs>
auto frame_cast(const ExpressionBase<Rhs> &rhs)
  -> TICK_FUNCTION_REQUIRES(internal::has_two_decorators<Rhs>{})(FrameCast<F1, F2, Rhs>) {
    return FrameCast<F1, F2, Rhs>{rhs.derived()};
}

/**
 * Forcibly changes the frame descriptors of an existing transform expression.
 *
 * @tparam F1, F2 the new frame descriptors
 */
template <typename F1, typename F2, typename Rhs>
auto frame_cast(ExpressionBase<Rhs> &&rhs) -> TICK_FUNCTION_REQUIRES(
  internal::has_two_decorators<Rhs>{})(FrameCast<F1, F2, Rhs &&>) {
    return FrameCast<F1, F2, Rhs &&>{std::move(rhs.derived())};
}


namespace internal {

// Traits shared by two- and three-framed variants
template <typename RhsDerived_, typename... Frames>
struct frame_cast_traits_base {
 private:
    using Derived = FrameCast<Frames..., RhsDerived_>;

 public:
    // Needed for unary expression concept
    template <typename NewRhs>
    using rebind = FrameCast<Frames..., NewRhs>;
    using RhsDerived = tmp::remove_cr_t<RhsDerived_>;

 private:
    // Type of the Rhs after applying its PreparedType (before evaluation)
    using RhsPrepared = typename traits<RhsDerived>::PreparedType;
    // Type of this expression template rebound to the prepared rhs
    using AdaptedType = rebind<RhsPrepared>;

 public:
    // We need no change in eval or conversions
    using Tag = leaf;  // leaf tag means we use identity eval and jacobianImpl
    using PreparedType = AdaptedType &&;
    using EvalType = eval_t<RhsDerived>;
    using UniqueLeaves = has_unique_leaves_unary<RhsDerived>;

    // We do override the output with new frames
    using OutputFunctor = WrapWithFrames<Frames...>;
};


// Traits for FrameCast with two frames
template <typename F1, typename F2, typename Rhs>
struct traits<FrameCast<F1, F2, Rhs>> : frame_cast_traits_base<Rhs, F1, F2> {
    using LeftFrame = F1;
    using RightFrame = F2;
};

// Traits for FrameCast with three frames
template <typename F1, typename F2, typename F3, typename Rhs>
struct traits<FrameCast<F1, F2, F3, Rhs>> : frame_cast_traits_base<Rhs, F1, F2, F3> {
    using LeftFrame = F1;
    using MiddleFrame = F2;
    using RightFrame = F3;
};

template <typename Rhs>
auto unframed_cast(const ExpressionBase<Rhs> &rhs) -> TICK_FUNCTION_REQUIRES(
  internal::has_two_decorators<Rhs>{})(FrameCast<NoFrame, NoFrame, Rhs>) {
    return FrameCast<NoFrame, NoFrame, Rhs>{rhs.derived()};
}

template <typename Rhs>
auto unframed_cast(const ExpressionBase<Rhs> &rhs) -> TICK_FUNCTION_REQUIRES(
  internal::has_three_decorators<Rhs>{})(FrameCast<NoFrame, NoFrame, NoFrame, Rhs>) {
    return FrameCast<NoFrame, NoFrame, NoFrame, Rhs>{rhs.derived()};
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_FRAME_CAST_HPP
