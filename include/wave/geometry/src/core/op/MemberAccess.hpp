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
    MemberAccess &&operator=(const typename Base::template BaseTmpl<Other> &other) && {
        // Skip evaluateTo and Evaluator for now because we need a non-const reference to
        // the leaf
        // @todo clean up
        auto &leaf = evalImpl(internal::get_expr_tag_t<Rhs>{}, this->rhs());
        auto block = internal::prepareLeafForOutput<MemberAccess>(Aux{}.get(leaf));
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


template <typename Aux, typename Rhs>
struct traits<MemberAccess<Aux, Rhs>> : unary_traits_base<MemberAccess<Aux, Rhs>> {
 public:
    using OutputFunctor = typename Aux::Frames;
    using TangentBlocks = typename traits<functor_return_t<Aux, Rhs>>::TangentBlocks;
    enum : int { TangentSize = traits<functor_return_t<Aux, Rhs>>::TangentSize };
};

/** Helper to make a member access expression.
 * Call from user-facing methods to reduce internal:: clutter */
template <typename Aux, typename Derived>
auto memberAccess(Derived &&expr) {
    return MemberAccess<Aux, arg_t<Derived>>{std::forward<Derived>(expr)};
}

template <typename Aux, typename Rhs>
auto evalImpl(expr<MemberAccess, Aux>, Rhs &&rhs) {
    // Evaluate to plain type to avoid storing a pointer to a cached result in Evaluator
    return Aux{}.get(std::forward<Rhs>(rhs)).eval();
}

template <typename Aux, typename Res, typename Rhs>
auto jacobianImpl(expr<MemberAccess, Aux>, const Res &res, const Rhs &rhs)
  -> BlockMatrix<Res, Rhs> {
    return Aux{}.jacobian(res, rhs);
}


}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_MEMBERACCESS_HPP
