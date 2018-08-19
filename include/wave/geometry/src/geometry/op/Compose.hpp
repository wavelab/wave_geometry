/**
 * @file
 */

#ifndef WAVE_GEOMETRY_COMPOSE_HPP
#define WAVE_GEOMETRY_COMPOSE_HPP

namespace wave {

/** An expression representing a product of two transformations.
 * It can apply to Rotations in SO(3) or RigidTransforms in SE(3) */
template <typename Lhs, typename Rhs>
struct Compose : internal::base_tmpl_t<Lhs, Rhs, Compose<Lhs, Rhs>>,
                 BinaryExpression<Compose<Lhs, Rhs>> {
 private:
    using Storage = BinaryExpression<Compose<Lhs, Rhs>>;

 public:
    // Inherit constructors from BinaryExpression
    using Storage::Storage;

    static_assert(std::is_same<RightFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Adjacent frames do not match");
};

/** A modified Compose equivalent to Compose<Rhs, Lhs> but with operands evaluated in
 * reverse order. It is not meant to be directly used, but forms a part of the composite
 * expression BoxPlus.
 */
template <typename Lhs, typename Rhs>
struct ComposeFlipped : internal::base_tmpl_t<Lhs, Rhs, ComposeFlipped<Lhs, Rhs>>,
                        BinaryExpression<ComposeFlipped<Lhs, Rhs>> {
 private:
    using Storage = BinaryExpression<ComposeFlipped<Lhs, Rhs>>;

 public:
    // Inherit constructors from BinaryExpression
    using Storage::Storage;

    static_assert(std::is_same<RightFrameOf<Rhs>, LeftFrameOf<Lhs>>(),
                  "Adjacent frames do not match");
};

namespace internal {

template <typename Lhs, typename Rhs>
struct traits<Compose<Lhs, Rhs>> : binary_traits_base<Compose<Lhs, Rhs>> {
    using OutputFunctor = WrapWithFrames<LeftFrameOf<Lhs>, RightFrameOf<Rhs>>;
};

template <typename Lhs, typename Rhs>
struct traits<ComposeFlipped<Lhs, Rhs>> : binary_traits_base<ComposeFlipped<Lhs, Rhs>> {
    using OutputFunctor = WrapWithFrames<LeftFrameOf<Rhs>, RightFrameOf<Lhs>>;
};

/** Left Jacobian of any composition is identity */
template <typename Val, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<Compose>,
                      const Val &,
                      const TransformBase<Lhs> &,
                      const TransformBase<Rhs> &) {
    return identity_t<Val>{};
}

/** Implements ComposeFlipped in terms of Compose*/
template <typename Lhs, typename Rhs>
auto evalImpl(expr<ComposeFlipped>,
              const TransformBase<Lhs> &lhs,
              const TransformBase<Rhs> &rhs)
  -> decltype(evalImpl(expr<Compose>{}, rhs.derived(), lhs.derived())) {
    return evalImpl(expr<Compose>{}, rhs.derived(), lhs.derived());
}

/** Evaluates Jacobian of ComposeFlipped in terms of Compose */
template <typename Val, typename Lhs, typename Rhs>
decltype(auto) rightJacobianImpl(expr<ComposeFlipped>,
                                 const Val &val,
                                 const TransformBase<Lhs> &lhs,
                                 const TransformBase<Rhs> &rhs) {
    return leftJacobianImpl(expr<Compose>{}, val.derived(), rhs.derived(), lhs.derived());
}

/** Evaluates Jacobian of ComposeFlipped in terms of Compose */
template <typename Val, typename Lhs, typename Rhs>
decltype(auto) leftJacobianImpl(expr<ComposeFlipped>,
                                const Val &val,
                                const TransformBase<Lhs> &lhs,
                                const TransformBase<Rhs> &rhs) {
    return rightJacobianImpl(
      expr<Compose>{}, val.derived(), rhs.derived(), lhs.derived());
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_COMPOSE_HPP
