/**
 * @file
 */

#ifndef WAVE_GEOMETRY_FRAMES_HPP
#define WAVE_GEOMETRY_FRAMES_HPP

namespace wave {

/** Wraps a leaf expression with frame descriptors.
 *
 * Framed is itself a leaf expression with the same evaluation behaviour as WrappedLeaf.
 *
 * @tparam Frames arbitrary typenames representing coordinate frames
 */
template <typename WrappedLeaf, typename... Frames>
class Framed;

template <typename WrappedLeaf, typename... Frames>
class Framed : public internal::base_tmpl_t<WrappedLeaf, Framed<WrappedLeaf, Frames...>>,
               public internal::FramedLeafAccess<Framed<WrappedLeaf, Frames...>> {
    TICK_TRAIT_CHECK(internal::is_leaf_expression<WrappedLeaf>);
    TICK_TRAIT_CHECK(internal::is_unframed<WrappedLeaf>);

    // Framed is a leaf expression which wraps another leaf expression. It is implemented
    // similarly to a UnaryExpressionBase holding WrappedLeaf by value, except with
    // different constructors

    // Store the wrapped leaf by value
    static_assert(!std::is_reference<WrappedLeaf>{},
                  "Template parameter to Framed cannot be a reference.");

    // Helper to determine whether to evaluate an Expression argument as Framed.
    // In the special case that we are Framed<T, NoFrame, NoFrame...>, we want to evaluate
    // the argument as Framed even if is_unframed<Arg> is true.
    static constexpr bool WeHaveFrames = !internal::is_unframed<Framed>::value;

    // Helper to determine whether we have a vector leaf and should enable the special
    // Scalar constructors. These constructors are needed to avoid "narrowing conversion"
    // warnings when forwarding calls made with integer literals such as Framed{0, 0, 0}.
    template <typename... Args>
    using special_constructor_applies =
      tmp::bool_constant<sizeof...(Args) == 3 &&
                         internal::is_vector_leaf<WrappedLeaf>{} &&
                         typename tmp::conjunction<std::is_arithmetic<Args>...>{}>;

    // Workaround for clang 3 bug. `Framed` must be treated as a template here
    using Tag = internal::expr<Framed::template Framed>;

    using Scalar = internal::scalar_t<WrappedLeaf>;

 private:
    // Store the wrapped leaf by value
    // This is up here so we can use it in decltype
    WrappedLeaf wrapped_leaf;

 public:
    /** Construct from an expression */
    template <
      typename OtherDerived,
      TICK_REQUIRES(WeHaveFrames and internal::same_frames<Framed, OtherDerived>{})>
    Framed(const ExpressionBase<OtherDerived> &other)
      // Evaluate to Framed and delegate to our copy or move constructor
      : Framed{wave::internal::evaluateTo<Framed>(other.derived())} {}

    /** Construct from an expression in the special case we have all NoFrame*/
    // The above constructor won't work in this case since evaluateTo will return an
    // unframed expression.
    template <typename OtherDerived,
              TICK_REQUIRES(!WeHaveFrames and
                            internal::same_frames<Framed, OtherDerived>{})>
    Framed(const ExpressionBase<OtherDerived> &other)
      // Evaluate to Framed and delegate to our copy or move constructor
      : wrapped_leaf{wave::internal::evaluateTo<WrappedLeaf>(other.derived())} {}

    // Forward args to rhs (version for one argument)
    // Disable if copy ctor would apply - https://stackoverflow.com/a/39646176

    template <class Arg,
              tmp::enable_if_t<!std::is_same<tmp::decay_t<Arg>, Framed>{} &&
                                 std::is_constructible<WrappedLeaf, Arg>{} &&
                                 (!internal::is_expression<tmp::remove_cr_t<Arg>>{}),
                               int> = 0>
    explicit Framed(Arg &&arg) : wrapped_leaf{std::forward<Arg>(arg)} {}

    // Forward args to rhs (version more than 1 argument)
    template <class... Args,
              tmp::enable_if_t<(sizeof...(Args) > 1) &&
                                 std::is_constructible<WrappedLeaf, Args...>::value &&
                                 !special_constructor_applies<Args...>{},
                               int> = 0>
    explicit Framed(Args &&... args) : wrapped_leaf{std::forward<Args>(args)...} {}

    // Special constructor for size-3 vectors, to allow Framed{0, 0, 0} without
    // narrowing warnings
    template <TICK_REQUIRES(internal::is_vector_leaf<WrappedLeaf>{})>
    explicit Framed(Scalar x, Scalar y, Scalar z) : wrapped_leaf{x, y, z} {}

    // Leave default csonstructors and assignment operators
    Framed() = default;
    Framed(const Framed &) = default;
    Framed(Framed &&) = default;
    Framed &operator=(const Framed &) = default;
    Framed &operator=(Framed &&) = default;

    /** Get value() of the wrapped leaf */
    auto value() const & -> decltype(wrapped_leaf.value()) {
        return this->wrapped_leaf.value();
    }

    auto value() & -> decltype(wrapped_leaf.value()) {
        return this->wrapped_leaf.value();
    }

    auto value() && -> decltype(std::move(wrapped_leaf).value()) {
        return std::move(this->wrapped_leaf).value();
    }

 private:
    // Allow the WrapWithFrames functor to contruct Framed from an unframed leaf
    friend struct internal::WrapWithFrames<Frames...>;

    // Allow our own FriendLeafAccess class to get the unframed leaf
    friend struct internal::FramedLeafAccessBase<Framed>;

    // Strip frames at the start of evaluation
    friend auto evalImpl(Tag, const Framed &f) -> const WrappedLeaf & {
        return f.wrapped_leaf;
    }
    friend auto evalImpl(Tag, Framed &f) -> WrappedLeaf & {
        return f.wrapped_leaf;
    }
    friend auto evalImpl(Tag, Framed &&f) -> WrappedLeaf && {
        return std::move(f).wrapped_leaf;
    }

    /** Construct from a leaf rvalue - only allowed for our friend functor which is used
     * internally by evaluateTo() itself */
    template <typename OtherDerived,
              tmp::enable_if_t<internal::is_leaf_expression<OtherDerived>{} &&
                                 internal::is_unframed<OtherDerived>{},
                               int> = 0>
    explicit Framed(ExpressionBase<OtherDerived> &&other)
        : wrapped_leaf{std::move(other.derived())} {}

    /** Construct from an unframed leaf - only allowed for our friend functor which is
     * used internally by evaluateTo() itself */
    template <typename OtherDerived,
              tmp::enable_if_t<internal::is_leaf_expression<OtherDerived>{} &&
                                 WeHaveFrames && internal::is_unframed<OtherDerived>{},
                               int> = 0>
    explicit Framed(const ExpressionBase<OtherDerived> &other)
        : wrapped_leaf{other.derived()} {}
};

namespace internal {

/** Meant to be inherited by FriendLeafAccess - provides unframed leaf access to children
 */
template <typename WrappedLeaf, typename... Frames>
struct FramedLeafAccessBase<Framed<WrappedLeaf, Frames...>> {
    using Derived = Framed<WrappedLeaf, Frames...>;

 protected:
    /** Return reference to this as derived object */
    inline WrappedLeaf &leaf() & noexcept {
        return static_cast<Derived *>(this)->wrapped_leaf;
    }
    /** Return reference to this as derived object */
    inline const WrappedLeaf &leaf() const &noexcept {
        return static_cast<Derived const *>(this)->wrapped_leaf;
    }
    /** Return reference to this as derived object, when this is rvalue */
    inline WrappedLeaf &&leaf() && noexcept {
        return std::move(*static_cast<Derived *>(this)).wrapped_leaf;
    }
};

/** Policy-like class template which can be specialized for Framed<Leaf, ...>
 * to give extra access methods like rotation() and translation().
 *
 * Framed<Leaf, ...> is intended to behave like Leaf, but it does not inherit Leaf
 * (otherwise, .derived() would not properly refer to Framed). Usually leaves have no
 * non-static, non-special methods other than value(), but sometimes it is desired: e.g.
 * for rotation() and translation() of a transform. Note these methods should have
 * different return types in a Framed<Transform> than in a Transform.
 *
 * In this case FramedLeafAccess allows us to to inject methods into Framed<Leaf, ...>
 * without specializing it entirely.
 *
 * This primary template does nothing.
 */
template <typename Derived, typename Enable>
struct FramedLeafAccess : FramedLeafAccessBase<Derived> {};


// Traits for Framed with two frames
template <typename WrappedLeaf, typename LeftFrame_, typename RightFrame_>
struct traits<Framed<WrappedLeaf, LeftFrame_, RightFrame_>>
  : frames_traits_base<WrappedLeaf, LeftFrame_, RightFrame_> {
    using LeftFrame = LeftFrame_;
    using RightFrame = RightFrame_;
};

// Traits for Framed with three frames
template <typename WrappedLeaf,
          typename LeftFrame_,
          typename MiddleFrame_,
          typename RightFrame_>
struct traits<Framed<WrappedLeaf, LeftFrame_, MiddleFrame_, RightFrame_>>
  : frames_traits_base<WrappedLeaf, LeftFrame_, MiddleFrame_, RightFrame_> {
    using LeftFrame = LeftFrame_;
    using MiddleFrame = MiddleFrame_;
    using RightFrame = RightFrame_;
};

// If someone asks to convert to Framed<Leaf, ...>, evaluate Convert<Leaf, Rhs> instead
template <typename Leaf, typename... Frames, typename Rhs>
auto evalImpl(expr<Convert, Framed<Leaf, Frames...>>, const Rhs &rhs)
  -> decltype(evalImpl(expr<Convert, Leaf>{}, rhs)) {
    return evalImpl(expr<Convert, Leaf>{}, rhs);
};


}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_FRAMES_HPP
