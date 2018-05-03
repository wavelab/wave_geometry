/**
 * @file
 */

#ifndef WAVE_GEOMETRY_IDENTITY_HPP
#define WAVE_GEOMETRY_IDENTITY_HPP

namespace wave {

/**
 * A wrapper expression representing an Identity transform
 */
template <typename Leaf>
class Identity : public internal::base_tmpl_t<Leaf, Identity<Leaf>> {
    TICK_TRAIT_CHECK(internal::is_leaf_expression<Leaf>);
    using Scalar = internal::scalar_t<Leaf>;

 public:
    /** Returns an identity translation expression */
    auto translation() const -> Zero<Translation<Eigen::Matrix<Scalar, 3, 1>>> {
        return Zero<Translation<Eigen::Matrix<Scalar, 3, 1>>>{};
    }

    /** Returns an identity rotation expression
     *
     * @returns an identity quaternion expression arbitrarily. @todo could return another
     * type based on our Leaf, but Identity<Quaternion> is easily converted to others.
     */
    auto rotation() const -> Identity<QuaternionRotation<Eigen::Quaternion<Scalar>>> {
        return Identity<QuaternionRotation<Eigen::Quaternion<Scalar>>>{};
    }

    // Satisfy the Leaf Expression concept
    auto value() const -> typename internal::eval_traits<Leaf>::ImplType {
        return eval(this->derived()).value();
    }
};

namespace internal {

template <typename Leaf>
struct traits<Identity<Leaf>> : traits<Leaf> {
    // Copy most traits from the wrapped Leaf, but override some
    using PreparedType = Identity<Leaf>;
    using UniqueLeaves = has_unique_leaves_leaf<Identity<Leaf>>;

    // In evaluation, we change the type
    using Tag = expr<Identity>;
    using EvalType = Identity<eval_t<Leaf>>;

    // Suggest conversion to the wrapped leaf type
    using ConvertTo = tmp::type_list<Leaf>;
};


// Implementations of operations on Identity vectors
// Convert from Identity<Leaf> to Leaf must be implemented for each wrapped leaf

// When evaluating, change Identity to the wrapped leaf's EvalType
// E.g. this allows stripping Framed<> during evaluation:
//     Identity<Framed<T>> -> Identity<T>
template <typename Leaf>
auto evalImpl(expr<Identity>, const Identity<Leaf> &) -> Identity<eval_t<Leaf>> {
    return Identity<eval_t<Leaf>>{};
}

template <typename Rhs>
auto evalImpl(expr<Inverse>, const Identity<Rhs> &rhs) -> const Identity<Rhs> & {
    return rhs;
};

template <typename Lhs, typename Rhs>
auto evalImpl(expr<Compose>, const TransformBase<Lhs> &lhs, const Identity<Rhs> &)
  -> const Lhs & {
    return lhs.derived();
};

template <typename Lhs, typename Rhs>
auto evalImpl(expr<Compose>, const Identity<Lhs> &, const TransformBase<Rhs> &rhs)
  -> const Rhs & {
    return rhs.derived();
};

template <typename Lhs, typename Rhs>
auto evalImpl(expr<Compose>, const Identity<Lhs> &lhs, const Identity<Rhs> &)
  -> const Identity<Lhs> & {
    return lhs;
};

template <typename Lhs, typename Rhs>
auto evalImpl(expr<Rotate>, const Identity<Lhs> &, const TranslationBase<Rhs> &rhs)
  -> const Rhs & {
    return rhs.derived();
};

template <typename Lhs, typename Rhs>
auto evalImpl(expr<Transform>, const Identity<Lhs> &, const TranslationBase<Rhs> &rhs)
  -> const Rhs & {
    return rhs.derived();
};

template <typename Rhs>
auto evalImpl(expr<LogMap>, const Identity<Rhs> &)
  -> Zero<typename traits<Rhs>::TangentType> {
    return Zero<typename traits<Rhs>::TangentType>{};
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_IDENTITY_HPP
