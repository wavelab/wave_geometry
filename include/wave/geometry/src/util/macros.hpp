/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_MACROS_HPP
#define WAVE_GEOMETRY_MACROS_HPP

#include <Eigen/Core>

#ifdef NDEBUG
#if EIGEN_COMP_CLANG
// Based on experiments, always_inline makes clang-4.0 faster, but not GCC
#define WAVE_STRONG_INLINE EIGEN_ALWAYS_INLINE
#else
#define WAVE_STRONG_INLINE EIGEN_STRONG_INLINE
#endif
#else
// When debugging don't force inlining
#define WAVE_STRONG_INLINE inline
#endif


/** We sometimes need to explicitly declare special member functions (copy constructors
 * and operator=) even when they should already be defaulted, to avoid a GCC bug
 * (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=63540) The default constructor is not
 * included. It is not related to the bug and it is more clear to show the default
 * constructor anyway.
 */
#define WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(Derived) \
    Derived(const Derived &) = default;               \
    Derived(Derived &&) = default;                    \
    Derived &operator=(const Derived &) = default;    \
    Derived &operator=(Derived &&) = default;

/** Generate overloaded function for three combinations of lvalue/rvalue paramaters
 * The first definition (both lvalues) is not included here as it is more clear to readers
 * (and greppable) if at least one definition is fully written out.
 */
#define WAVE_OVERLOAD_FUNCTION_FOR_RVALUES_IMPL(                                         \
  FuncName, ExprName, LhsBase, RhsBase, ...)                                             \
    template <__VA_ARGS__>                                                               \
    auto FuncName(LhsBase<L> &&lhs, const RhsBase<R> &rhs) {                             \
        return ExprName<internal::arg_t<L>, R>{std::move(lhs).derived(), rhs.derived()}; \
    }                                                                                    \
    template <__VA_ARGS__>                                                               \
    auto FuncName(const LhsBase<L> &lhs, RhsBase<R> &&rhs) {                             \
        return ExprName<L, internal::arg_t<R>>{lhs.derived(), std::move(rhs).derived()}; \
    }                                                                                    \
    template <__VA_ARGS__>                                                               \
    auto FuncName(LhsBase<L> &&lhs, RhsBase<R> &&rhs) {                                  \
        return ExprName<internal::arg_t<L>, internal::arg_t<R>>{                         \
          std::move(lhs).derived(), std::move(rhs).derived()};                           \
    }

#define WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(FuncName, ExprName, LhsBase, RhsBase) \
    WAVE_OVERLOAD_FUNCTION_FOR_RVALUES_IMPL(                                     \
      FuncName, ExprName, LhsBase, RhsBase, typename L, typename R)

#define WAVE_OVERLOAD_FUNCTION_FOR_RVALUES_REQ(             \
  FuncName, ExprName, LhsBase, RhsBase, ExtraTemplateParam) \
    WAVE_OVERLOAD_FUNCTION_FOR_RVALUES_IMPL(                \
      FuncName, ExprName, LhsBase, RhsBase, typename L, typename R, ExtraTemplateParam)


/** Generate overloaded unary function for an rvalue argument. The first definition
 * (lvalues argument) is not included here as it is more clear to readers (and greppable)
 * if at least one definition is fully written out.
 */
#define WAVE_OVERLOAD_FUNCTION_FOR_RVALUE(FuncName, ExprName, RhsBase) \
    template <typename R>                                              \
    auto FuncName(RhsBase<R> &&rhs)->ExprName<internal::arg_t<R>> {    \
        return ExprName<internal::arg_t<R>>{std::move(rhs).derived()}; \
    }


/** Generate overloaded binary functions where one side is a plain scalar to be wrapped */
#define WAVE_OVERLOAD_OPERATORS_FOR_SCALAR_LEFT(OpSymbol, RhsBase)                     \
    template <typename L, typename R, TICK_REQUIRES(internal::is_scalar<L>{})>         \
    auto operator OpSymbol(L &&lhs, const RhsBase<R> &rhs) {                           \
        return internal::wrapInputScalar(std::forward<L>(lhs)) OpSymbol rhs.derived(); \
    }                                                                                  \
                                                                                       \
    template <typename L, typename R, TICK_REQUIRES(internal::is_scalar<L>{})>         \
    auto operator OpSymbol(L &&lhs, RhsBase<R> &&rhs) {                                \
        return internal::wrapInputScalar(std::forward<L>(lhs)) OpSymbol std::move(rhs) \
          .derived();                                                                  \
    }

/** Generate overloaded binary functions where one side is a plain scalar to be wrapped */
#define WAVE_OVERLOAD_OPERATORS_FOR_SCALAR_RIGHT(OpSymbol, LhsBase)                    \
    template <typename L, typename R, TICK_REQUIRES(internal::is_scalar<R>{})>         \
    auto operator OpSymbol(const LhsBase<L> &lhs, R &&rhs) {                           \
        return lhs.derived() OpSymbol internal::wrapInputScalar(std::forward<R>(rhs)); \
    }                                                                                  \
                                                                                       \
    template <typename L, typename R, TICK_REQUIRES(internal::is_scalar<R>{})>         \
    auto operator OpSymbol(LhsBase<L> &&lhs, R &&rhs) {                                \
        return std::move(lhs).derived()                                                \
          OpSymbol internal::wrapInputScalar(std::forward<R>(rhs));                    \
    }

/** Generate overloaded binary functions where one side is a plain scalar to be wrapped
 * (for scalar on both left and right) */
#define WAVE_OVERLOAD_OPERATORS_FOR_SCALAR(OpSymbol, OtherBase)  \
    WAVE_OVERLOAD_OPERATORS_FOR_SCALAR_LEFT(OpSymbol, OtherBase) \
    WAVE_OVERLOAD_OPERATORS_FOR_SCALAR_RIGHT(OpSymbol, OtherBase)

#endif  // WAVE_GEOMETRY_MACROS_HPP
