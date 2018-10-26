/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_MEMBERACCESS_HPP
#define WAVE_GEOMETRY_MEMBERACCESS_HPP

namespace wave {
namespace internal {

template <typename Aux, typename Rhs>
using functor_return_t = decltype(std::declval<Aux>().get(std::declval<eval_t<Rhs>>()));

template <typename Aux, typename Rhs, typename Derived>
using aux_base_tmpl_t = base_tmpl_t<functor_return_t<Aux, Rhs>, Derived>;

}  // namespace internal

/**
 * Expression for deferred call to a leaf member function.
 *
 * @tparam Aux an auxilliary traits struct describing the deferred call.
 *
 * Every unique use of this expression needs its own implementation function of the form
 * evalImpl(Aux, const RhsDerived &) -> typename Res::type
 *
 * @todo maybe replace the evalImpl with a lambda passed to this expression, or something.
 */
template <typename Aux, typename Rhs>
struct MemberAccess final : internal::aux_base_tmpl_t<Aux, Rhs, MemberAccess<Aux, Rhs>>,
                            internal::unary_storage_for<MemberAccess<Aux, Rhs>> {
 private:
    using Base = internal::aux_base_tmpl_t<Aux, Rhs, MemberAccess<Aux, Rhs>>;
    using OutType = internal::functor_return_t<Aux, Rhs>;
    using Storage = internal::unary_storage_for<MemberAccess>;

 public:
    using Storage::Storage;

    /** Assigns to the subobject from an expression of the matching space.
     *
     * Enabled if Rhs is detected to be assignable, but might fail somewhere in Eigen if
     * the underlying type of Rhs is not a plain Eigen object.
     *
     * Only allowed when this is an rvalue: we want to allow `a.translation() = x` but
     * avoid the confusion of `auto t = a.translation(); t = x;`
     */
    template <typename Other,
              std::enable_if_t<std::is_assignable<OutType &, const Other &>{} &&
                                 std::is_lvalue_reference<Rhs>{},
                               bool> = true>
    MemberAccess &&operator=(typename Base::template BaseTmpl<Other> &other) && {
        // Skip evaluateTo and Evaluator for now because we need a non-const reference to
        // the leaf
        // @todo clean up
        auto &leaf = evalImpl(internal::get_expr_tag_t<Rhs>{}, this->rhs());
        auto block = Aux{}.get(leaf);
        block = other.derived();
        return std::move(*this);
    }

    // Overload for rvalue
    template <typename Other,
              std::enable_if_t<std::is_assignable<OutType &, Other &&>{} &&
                                 std::is_lvalue_reference<Rhs>{},
                               bool> = true>
    MemberAccess &&operator=(typename Base::template BaseTmpl<Other> &&other) {
        auto &leaf = evalImpl(internal::get_expr_tag_t<Rhs>{}, this->rhs());
        auto block = internal::prepareLeafForOutput<MemberAccess>(Aux{}.get(leaf));
        block = std::move(other).derived();
        return std::move(*this);
    }
};

namespace internal {

template <typename Aux>
struct member_access_tag {};

template <typename Aux, typename Rhs>
struct traits<MemberAccess<Aux, Rhs>>
    : unary_traits_base_tag<MemberAccess<Aux, Rhs>, member_access_tag<Aux>> {
 public:
    using OutputFunctor = typename Aux::Frames;
};

/** Helper to make a member access expression.
 * Call from user-facing methods to reduce internal:: clutter */
template <typename Aux, typename Derived>
auto memberAccess(Derived &&expr) {
    return MemberAccess<Aux, arg_t<Derived>>{std::forward<Derived>(expr)};
}


template <typename Aux, typename Rhs>
auto evalImpl(member_access_tag<Aux>, Rhs &&rhs) {
    return Aux{}.get(std::forward<Rhs>(rhs));
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_MEMBERACCESS_HPP
