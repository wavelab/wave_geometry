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
struct PrepareExpr<Derived, enable_if_leaf_or_nullary_t<tmp::remove_cr_t<Derived>>> {
    // Note if we were to write run(Derived&& leaf), it would not be a forwarding
    // reference (aka "universal reference"), so we introduce a new template parameter
    // here. For the leaf case specifically we could figure out the type of the argument
    // to run based on Derived, but we keep it consistent with the other cases.
    template <typename T>
    static auto run(T &&leaf) -> T {
        return std::forward<T>(leaf);
    }
};

template <typename Derived>
struct PrepareExpr<Derived, enable_if_unary_t<Derived>> {
    using PreparedType = typename traits<Derived>::PreparedType;
    using Rhs = typename traits<Derived>::RhsDerived;

    template <typename T>
    static auto run(T &&unary) -> PreparedType {
        return PreparedType{
          PrepareExpr<Rhs>::run(std::forward<T>(unary).derived().rhs())};
    }
};

template <typename Derived>
struct PrepareExpr<Derived, enable_if_binary_t<Derived>> {
    using PreparedType = typename traits<Derived>::PreparedType;
    using Lhs = typename traits<Derived>::LhsDerived;
    using Rhs = typename traits<Derived>::RhsDerived;

    template <typename T>
    static auto run(T &&binary) -> PreparedType {
        return PreparedType{
          PrepareExpr<Lhs>::run(std::forward<T>(binary).derived().lhs()),
          PrepareExpr<Rhs>::run(std::forward<T>(binary).derived().rhs())};
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
