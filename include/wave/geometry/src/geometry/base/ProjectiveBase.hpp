/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_PROJECTIVEBASE_HPP
#define WAVE_GEOMETRY_PROJECTIVEBASE_HPP

namespace wave {

/** Represents an element of projective space
 */
template <typename Derived>
struct ProjectiveBase : ExpressionBase<Derived> {
 private:
    using Scalar = internal::scalar_t<Derived>;
    using OutputType = internal::plain_output_t<Derived>;

 public:
    /** Produce a random spherically normalized homogeneous point */
    static auto Random() -> OutputType {
        return ::wave::Random<OutputType>{}.eval();
    }

    /** Fuzzy comparison of two homogeneous point elements
     *
     * For now, performs box minus of the points and compares the resulting vector
     * to zero - see Eigen::DenseBase::isZero()
     *
     * @todo optimize. A faster per-parametrization comparison (e.g. are two quaternions
     * close?) may be desired. It is harder to implement for non-unique representations.
     */
    template <typename R, TICK_REQUIRES(internal::same_base_tmpl_i<Derived, R>{})>
    bool isApprox(
      const ProjectiveBase<R> &rhs,
      const Scalar &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        const auto diff = eval(internal::unframed_cast(this->derived()) -
                               internal::unframed_cast(rhs.derived()));
        return diff.value().isZero(prec);
    }
};


/** Applies the manifold minus operator to two homogeneous elements
 */
template <typename L, typename R>
auto operator-(const ProjectiveBase<L> &lhs, const ProjectiveBase<R> &rhs) {
    return BoxMinus<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                  rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator-, BoxMinus, ProjectiveBase, ProjectiveBase)

/** Applies the manifold plus operator to a transform element
 */
template <typename L, typename R, TICK_REQUIRES(internal::rhs_is_tangent_of_lhs<L, R>{})>
auto operator+(const ProjectiveBase<L> &lhs, const VectorBase<R> &rhs) {
    return HomPlus<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                 rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES_REQ(operator+,
                                       HomPlus,
                                       ProjectiveBase,
                                       VectorBase,
                                       TICK_REQUIRES(
                                         internal::rhs_is_tangent_of_lhs<L, R>{}))


}  // namespace wave

#endif  // WAVE_GEOMETRY_PROJECTIVEBASE_HPP
