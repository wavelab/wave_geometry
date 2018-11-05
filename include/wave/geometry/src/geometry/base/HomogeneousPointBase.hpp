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
    template <typename T>
    using BaseTmpl = HomogeneousPointBase<T>;
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
/** Implementation of Random for a rotation leaf
 *
 * Produces a random spherically normalized homogeneous point
 */
template <typename Leaf, typename Rhs>
auto evalImpl(expr<Random, Leaf>, const HomogeneousPointBase<Rhs> &) {
    return Leaf{traits<Leaf>::ImplType::Random().normalized()};
}

/** Implements perturbation of a homogeneous point  */
template <typename Lhs, typename Rhs>
auto evalImpl(expr<PerturbPlus>,
              const HomogeneousPointBase<Lhs> &lhs,
              const TranslationBase<Rhs> &rhs) {
    const auto &p = lhs.derived().value();
    const auto &v = rhs.derived().value();
    return
      typename traits<Lhs>::PlainType{p.x() + v.x(), p.y() + v.y(), p.z() + v.z(), p.w()};
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

}  // namespace internal

}  // namespace wave

#endif  // WAVE_GEOMETRY_HOMOGENEOUSPOINTBASE_HPP
