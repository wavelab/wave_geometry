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
template <typename Derived, typename Enable = void>
struct PrepareExpr;

template <typename Derived>
struct PrepareExpr<Derived,
                   enable_if_leaf_nullary_or_scalar_t<tmp::remove_cr_t<Derived>>> {
    using Leaf = tmp::remove_cr_t<Derived>;

    static auto run(const Leaf &leaf) -> const Leaf & {
        return leaf;
    }
};

template <typename Derived>
struct PrepareExpr<Derived, enable_if_unary_t<Derived>> {
    using PreparedType = typename traits<Derived>::PreparedType;
    using Rhs = typename traits<Derived>::RhsDerived;

    template <typename T>
    static auto run(const T &unary) -> PreparedType {
        return PreparedType{PrepareExpr<Rhs>::run(unary.derived().rhs())};
    }
};

template <typename Derived>
struct PrepareExpr<Derived, enable_if_binary_t<Derived>> {
    using PreparedType = typename traits<Derived>::PreparedType;
    using Lhs = typename traits<Derived>::LhsDerived;
    using Rhs = typename traits<Derived>::RhsDerived;

    template <typename T>
    static auto run(const T &binary) -> PreparedType {
        return PreparedType{PrepareExpr<Lhs>::run(binary.derived().lhs()),
                            PrepareExpr<Rhs>::run(binary.derived().rhs())};
    }
};

// Unary and binary expressions: always stored by value, so dicsard &&
template <typename Derived>
struct PrepareExpr<Derived &&, enable_if_unary_t<Derived>> : PrepareExpr<Derived> {};

template <typename Derived>
struct PrepareExpr<Derived &&, enable_if_binary_t<Derived>> : PrepareExpr<Derived> {};

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
