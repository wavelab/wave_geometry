/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_TRANSFORMBASE_HPP
#define WAVE_GEOMETRY_TRANSFORMBASE_HPP

namespace wave {

/** Base class for transform-like expressions including SO(3) (RotationBase) and SE(3)
 * (RigidTransformBase)
 */
template <typename Derived>
class TransformBase : public ExpressionBase<Derived> {
    using Scalar = internal::scalar_t<Derived>;
    using OutputType = internal::plain_output_t<Derived>;

 public:
    /** Produce a random element */
    static auto Random() -> OutputType {
        return ::wave::Random<OutputType>{}.eval();
    }

    /** Produce an expression representing an identity element */
    static auto Identity() -> ::wave::Identity<OutputType> {
        return ::wave::Identity<OutputType>{};
    }

    /** Fuzzy comparison of two transform elements
     *
     * For now, performs box minus of the rotations and compares the resulting vector
     * to zero - see Eigen::DenseBase::isZero()
     *
     * @todo optimize. A faster per-parametrization comparison (e.g. are two quaternions
     * close?) may be desired. It is harder to implement for non-unique representations.
     */
    template <typename R, TICK_REQUIRES(internal::same_base_tmpl_i<Derived, R>{})>
    bool isApprox(
      const TransformBase<R> &rhs,
      const Scalar &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        const auto diff = eval(internal::unframed_cast(this->derived()) -
                               internal::unframed_cast(rhs.derived()));
        return diff.value().isZero(prec);
    }
};

/** Gets inverse of a transform */
template <typename Derived>
auto inverse(const TransformBase<Derived> &rhs) -> Inverse<Derived> {
    return Inverse<Derived>{rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUE(inverse, Inverse, TransformBase)

/** Takes logarithmic map of a transform
 *
 * @f[ SO(3) \to \mathbb{R}^3 @f]
 */
template <typename R>
auto log(const TransformBase<R> &rhs) -> LogMap<RightFrameOf<R>, R> {
    return LogMap<RightFrameOf<R>, R>{rhs.derived()};
}
// Overload for rvalue
template <typename R>
auto log(TransformBase<R> &&rhs) -> LogMap<RightFrameOf<R>, internal::arg_t<R>> {
    return LogMap<RightFrameOf<R>, internal::arg_t<R>>{std::move(rhs).derived()};
}

/** Takes logarithmic map of a transform, with extra frame
 *
 * @f[ SO(3) \to \mathbb{R}^3 @f]
 */
template <typename ExtraFrame, typename R>
auto logFramed(const TransformBase<R> &rhs) -> LogMap<ExtraFrame, R> {
    return LogMap<ExtraFrame, R>{rhs.derived()};
}
// Overload for rvalue
template <typename ExtraFrame, typename R>
auto logFramed(TransformBase<R> &&rhs) -> LogMap<ExtraFrame, internal::arg_t<R>> {
    return LogMap<ExtraFrame, internal::arg_t<R>>{std::move(rhs).derived()};
}

/** Composes two transforms
 *
 * @f[ SO(3) \times SO(3) \to SO(3) @f]
 */
template <typename L, typename R>
auto operator*(const TransformBase<L> &lhs, const TransformBase<R> &rhs)
  -> Compose<L, R> {
    return Compose<L, R>{lhs.derived(), rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator*, Compose, TransformBase, TransformBase)

/** Applies the manifold minus operator to two transform elements
 *
 * @f[ SO(3) \times SO(3) \to so(3) @f]
 */
template <typename L, typename R>
auto operator-(const TransformBase<L> &lhs, const TransformBase<R> &rhs)
  -> BoxMinus<L, R> {
    return BoxMinus<L, R>{lhs.derived(), rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator-, BoxMinus, TransformBase, TransformBase)

/** Applies the manifold plus operator to a transform element
 *
 * @f[ SO(3) \times so(3) \to SO(3) @f]
 */
template <typename L, typename R, TICK_REQUIRES(internal::rhs_is_tangent_of_lhs<L, R>{})>
auto operator+(const TransformBase<L> &lhs, const VectorBase<R> &rhs) -> BoxPlus<L, R> {
    return BoxPlus<L, R>{lhs.derived(), rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES_REQ(operator+,
                                       BoxPlus,
                                       TransformBase,
                                       VectorBase,
                                       TICK_REQUIRES(
                                         internal::rhs_is_tangent_of_lhs<L, R>{}))

}  // namespace wave

#endif  // WAVE_GEOMETRY_TRANSFORMBASE_HPP
