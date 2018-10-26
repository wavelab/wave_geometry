/**
 * @file
 */

#ifndef WAVE_GEOMETRY_PREPAREEXPR_HPP
#define WAVE_GEOMETRY_PREPAREEXPR_HPP

namespace wave {
namespace internal {

/**
 * Transforms the expression tree: keeps leaves intact, but converts unary and binary
 * expressions to their traits::PreparedType
 */
template <typename Derived,
          enable_if_leaf_or_scalar_t<tmp::remove_cr_t<Derived>, bool> = true>
static auto prepareExpr(adl, const Derived &leaf) -> const Derived & {
    return leaf;
}

template <typename Derived, enable_if_unary_t<tmp::remove_cr_t<Derived>, bool> = true>
static auto prepareExpr(adl, const Derived &unary) {
    using OutType = tmp::remove_cr_t<typename traits<Derived>::PreparedType>;
    return OutType{prepareExpr(adl{}, unary.rhs())};
}

template <typename Derived, enable_if_binary_t<tmp::remove_cr_t<Derived>, bool> = true>
static auto prepareExpr(adl, const Derived &binary) {
    using OutType = tmp::remove_cr_t<typename traits<Derived>::PreparedType>;
    return OutType{prepareExpr(adl{}, binary.lhs()), prepareExpr(adl{}, binary.rhs())};
}


template <typename Derived, size_t... Is>
static auto prepareExprExpand(const Derived &nary, std::index_sequence<Is...>) {
    using OutType = tmp::remove_cr_t<typename traits<Derived>::PreparedType>;
    return OutType{prepareExpr(adl{}, nary.template get<Is>())...};
}

template <typename Derived, enable_if_nary_t<tmp::remove_cr_t<Derived>, bool> = true>
static auto prepareExpr(adl, const Derived &nary) {
    using Indices = std::make_index_sequence<traits<Derived>::CompoundSize>;
    return prepareExprExpand(nary, Indices{});
}

/** Functor which returns the given argument
 * To be used an OutputFunctor */
struct IdentityFunctor {
    template <typename Arg>
    constexpr auto operator()(Arg &&arg) const noexcept -> Arg && {
        return std::forward<Arg>(arg);
    }
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_PREPAREEXPR_HPP
