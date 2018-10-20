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
template <typename Derived>
struct PrepareExpr<Derived, enable_if_leaf_or_scalar_t<tmp::remove_cr_t<Derived>>> {
    using Leaf = tmp::remove_cr_t<Derived>;

    static auto run(const Leaf &leaf) -> const Leaf & {
        return leaf;
    }
};

template <typename Derived>
struct PrepareExpr<Derived, enable_if_unary_t<tmp::remove_cr_t<Derived>>> {
    using OutType = tmp::remove_cr_t<typename traits<Derived>::PreparedType>;
    using Rhs = typename traits<Derived>::RhsDerived;

    static auto run(const Derived &unary) {
        return OutType{PrepareExpr<Rhs>::run(unary.derived().rhs())};
    }
};

template <typename Derived>
struct PrepareExpr<Derived, enable_if_binary_t<tmp::remove_cr_t<Derived>>> {
    using OutType = tmp::remove_cr_t<typename traits<Derived>::PreparedType>;
    using Lhs = typename traits<Derived>::LhsDerived;
    using Rhs = typename traits<Derived>::RhsDerived;

    static auto run(const Derived &binary) {
        return OutType{PrepareExpr<Lhs>::run(binary.derived().lhs()),
                       PrepareExpr<Rhs>::run(binary.derived().rhs())};
    }
};

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
