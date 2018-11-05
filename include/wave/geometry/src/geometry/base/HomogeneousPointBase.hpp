/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_HOMOGENEOUSPOINTBASE_HPP
#define WAVE_GEOMETRY_HOMOGENEOUSPOINTBASE_HPP

namespace wave {

/** Base class for homogeneous points in P^3 */
template <typename Derived>
struct HomogeneousPointBase : public ProjectiveBase<Derived> {
 private:
    using OutputType = internal::plain_output_t<Derived>;

 public:
    template <typename T>
    using BaseTmpl = HomogeneousPointBase<T>;

    /** Returns an expression representing the homogeneous point (0, 0, 0, 1) */
    static auto Identity() {
        return ::wave::Identity<OutputType>();
    }

    /** Returns an expression representing the homogeneous point (0, 0, 0, 1) */
    static auto Zero() {
        return Identity();
    }
};

/** Applies a small perturbation to a homogeneous point
 */
template <typename L, typename R, TICK_REQUIRES(internal::rhs_is_tangent_of_lhs<L, R>{})>
auto perturb(const HomogeneousPointBase<L> &lhs, const TranslationBase<R> &rhs) {
    return PerturbPlus<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                     rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES_REQ(
  perturb,
  PerturbPlus,
  HomogeneousPointBase,
  TranslationBase,
  TICK_REQUIRES(internal::rhs_is_tangent_of_lhs<L, R>{}))

/** Recovers the perturbation between two close homogeneous points
 */
template <typename L, typename R>
auto deperturb(const HomogeneousPointBase<L> &lhs, const HomogeneousPointBase<R> &rhs) {
    return PerturbMinus<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                      rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(deperturb,
                                   PerturbMinus,
                                   HomogeneousPointBase,
                                   HomogeneousPointBase)

namespace internal {
/** Implements Random for homogeneous point leaves
 *
 * Produces a random spherically normalized homogeneous point
 */
template <typename Leaf, typename Rhs>
auto evalImpl(expr<Random, Leaf>, const HomogeneousPointBase<Rhs> &) {
    return Leaf{::wave::randomQuaternion<scalar_t<Rhs>>().coeffs()};
}

/** Implements Identity (zero point) for homogeneous points */
template <typename ToLeaf,
          typename FromLeaf,
          std::enable_if_t<std::is_base_of<HomogeneousPointBase<ToLeaf>, ToLeaf>{} &&
                             std::is_base_of<HomogeneousPointBase<FromLeaf>, FromLeaf>{},
                           bool> = 0>
auto evalImpl(expr<Convert, ToLeaf>, const Identity<FromLeaf> &) {
    using S = typename traits<ToLeaf>::Scalar;
    return ToLeaf{S{0}, S{0}, S{0}, S{1}};
}

/** Implements perturbation of a homogeneous point  */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<PerturbPlus>,
              const HomogeneousPointBase<Lhs> &lhs,
              const TranslationBase<Rhs> &rhs) {
    const auto &p = lhs.derived().value();
    const auto &v = rhs.derived().value();
    return plain_output_t<Lhs>{p.x() + v.x(), p.y() + v.y(), p.z() + v.z(), p.w()};
}

/** Jacobian of perturbation of a homogeneous point wrt the point */
template <typename Val, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<PerturbPlus>,
                      const HomogeneousPointBase<Val> &,
                      const HomogeneousPointBase<Lhs> &,
                      const TranslationBase<Rhs> &) {
    return identity_t<Val>{};
}

/** Jacobian of perturbation of a homogeneous point wrt the perturbation */
template <typename Val, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<PerturbPlus>,
                       const HomogeneousPointBase<Val> &,
                       const HomogeneousPointBase<Lhs> &,
                       const TranslationBase<Rhs> &) {
    return jacobian_t<Val, Rhs>::Identity();
}

/** Implements deperturbation of a homogeneous point  */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<PerturbMinus>,
              const HomogeneousPointBase<Lhs> &lhs,
              const HomogeneousPointBase<Rhs> &rhs) {
    const auto &a = lhs.derived().value();
    const auto &b = rhs.derived().value();
    return typename traits<Lhs>::TangentType{a.x() - b.x(), a.y() - b.y(), a.z() - b.z()};
}

/** Left Jacobian of deperturbation of a homogeneous point */
template <typename Val, typename Lhs, typename Rhs>
auto leftJacobianImpl(expr<PerturbMinus>,
                      const TranslationBase<Val> &,
                      const HomogeneousPointBase<Lhs> &,
                      const HomogeneousPointBase<Rhs> &) {
    return jacobian_t<Val, Lhs>::Identity();
}

/** Right Jacobian of deperturbation of a homogeneous point */
template <typename Val, typename Lhs, typename Rhs>
auto rightJacobianImpl(expr<PerturbMinus>,
                       const TranslationBase<Val> &,
                       const HomogeneousPointBase<Lhs> &,
                       const HomogeneousPointBase<Rhs> &) {
    return -jacobian_t<Val, Rhs>::Identity();
}

}  // namespace internal

}  // namespace wave

#endif  // WAVE_GEOMETRY_HOMOGENEOUSPOINTBASE_HPP
