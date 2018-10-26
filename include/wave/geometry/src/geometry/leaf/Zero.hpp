/**
 * @file
 */

#ifndef WAVE_GEOMETRY_ZERO_HPP
#define WAVE_GEOMETRY_ZERO_HPP

namespace wave {

/**
 * A wrapper expression representing a zero vector value
 */
template <typename Leaf>
class Zero : public internal::base_tmpl_t<Leaf, Zero<Leaf>> {
    TICK_TRAIT_CHECK(internal::is_vector_leaf<Leaf>);
    using ImplType = typename internal::traits<Leaf>::ImplType;
    using Scalar = typename Eigen::internal::traits<ImplType>::Scalar;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

 public:
    template <TICK_REQUIRES(internal::is_derived_twist<Leaf>{})>
    Zero<RelativeRotation<Vec3>> rotation() const {
        return Zero<RelativeRotation<Vec3>>{};
    }

    template <TICK_REQUIRES(internal::is_derived_twist<Leaf>{})>
    Zero<Translation<Vec3>> translation() const {
        return Zero<Translation<Vec3>>{};
    }

    // Satisfy the Leaf Expression concept
    auto value() const -> ImplType {
        return ImplType::Zero();
    }
};

namespace internal {

template <typename Leaf>
struct traits<Zero<Leaf>> : traits<Leaf> {
    // Copy most traits from the wrapped Leaf, but override some
    using PreparedType = const Zero<Leaf> &;
    using UniqueLeaves = has_unique_leaves_leaf<Zero<Leaf>>;

    // In evaluation, we change the type
    using Tag = expr<Zero>;
    using EvalType = Zero<eval_t<Leaf>>;

    // Suggest conversion to the wrapped leaf type
    using ConvertTo = tmp::type_list<Leaf>;
};


// Implementations of operations on Zero vectors

// When evaluating, change Zero to the wrapped leaf's EvalType
// E.g. this allows stripping Framed<> during evaluation: Zero<Framed<T>> -> Zero<T>
template <typename Leaf>
auto evalImpl(expr<Zero>, const Zero<Leaf> &) -> Zero<eval_t<Leaf>> {
    return Zero<eval_t<Leaf>>{};
}

template <typename Leaf>
auto evalImpl(expr<Convert, Leaf>, const Zero<Leaf> &) -> Leaf {
    return Leaf{traits<Leaf>::ImplType::Zero()};
}

template <typename Lhs, typename Rhs>
decltype(auto) evalImpl(expr<Sum>, const VectorBase<Lhs> &lhs, const Zero<Rhs> &) {
    return lhs.derived();
}

template <typename Lhs, typename Rhs>
decltype(auto) evalImpl(expr<Sum>, const Zero<Lhs> &, const VectorBase<Rhs> &rhs) {
    return rhs.derived();
}

template <typename Lhs, typename Rhs>
decltype(auto) evalImpl(expr<Sum>, const Zero<Lhs> &lhs, const Zero<Rhs> &) {
    return lhs;
}

template <typename Rhs>
decltype(auto) evalImpl(expr<Minus>, const Zero<Rhs> &rhs) {
    return rhs;
}

template <typename Rhs>
auto evalImpl(expr<ExpMap>, const Zero<Rhs> &) {
    return Identity<typename traits<Rhs>::ExpType>{};
}

/** Jacobian of exp map of a zero element */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<ExpMap>, const TransformBase<Val> &, const Zero<Rhs> &) {
    return identity_t<Rhs>{};
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_ZERO_HPP
