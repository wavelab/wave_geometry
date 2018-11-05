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
};


/** Applies the manifold minus operator to two homogeneous elements
 */
template <typename L, typename R>
auto operator-(const ProjectiveBase<L> &lhs, const ProjectiveBase<R> &rhs) {
    return HomMinus<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                  rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator-, HomMinus, ProjectiveBase, ProjectiveBase)

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
