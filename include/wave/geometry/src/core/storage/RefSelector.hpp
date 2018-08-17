/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_REFSELECTOR_HPP

namespace wave {
namespace internal {
/** Helper to choose storage type (reference or value) for expressions.
 *
 * T should be a wave_geometry expression type.
 *
 * If the input T is an rvalue reference, we want to store it by value. If T is an lvalue
 * reference, we store it by lvalue reference.  Otherwise, we want to see whether it
 * refers to a leaf or to a lightweight expression, and store it by value if the latter
 * (as in Eigen).
 *
 * | input  | output      |
 * |--------|-------------|
 * | Expr   | Expr        |
 * | Expr&& | Expr        |
 * | Leaf   | const Leaf& |
 * | Leaf&& | Leaf        |
 *
 */
template <typename T>
using wave_ref_sel_t =
  tmp::conditional_t<std::is_rvalue_reference<T>{},
                     tmp::remove_cr_t<T>,
                     tmp::conditional_t<is_leaf_expression<tmp::remove_cr_t<T>>::value,
                                        const T &,
                                        tmp::remove_cr_t<T>>>;

/** Helper to choose cache storage type (reference or value) for Jacobian evaluators.
 *
 * We pass the result of decltype() to this template. T should be an Eigen matrix
 * expression.
 *
 * If the input T is an lvalue reference, we want to see whether it refers to a leaf or
 * to a lightweight expression, and store it by value if the latter (as in Eigen).
 *
 * Otherwise if the input T is an rvalue, we want to store it by value.
 **/
template <typename T>
using jac_ref_sel_t =
  tmp::conditional_t<std::is_lvalue_reference<T>{},
                     typename Eigen::internal::ref_selector<tmp::remove_cr_t<T>>::type,
                     const tmp::remove_cr_t<T>>;

/** Helper to choose template argument for an expression template when an expression is
 * passed into a function with a forwaring reference.
 *
 * For example, template <typename T> auto inverse(T&& expr) -> Inverse<arg_t<T>>
 *
 * If T is const Leaf &, we want Inverse<T>
 * If T is Leaf, we want Inverse<T&&>
 * If T is const Unary & or Unary, we want Inverse<T>
 *
 * It has the same goal as wave_ref_sel_t, but is meant to be used in a user-facing
 * function, e.g. operator+(lhs, rhs). It should either give an non-ref non-const type or
 * an rvalue reference, to be used later by wave_ref_sel_t.
 */
template <typename T>
using arg_t = tmp::conditional_t<
  is_leaf_expression<tmp::remove_cr_t<T>>{},
  tmp::conditional_t<std::is_reference<T>{}, tmp::remove_cr_t<T>, T &&>,
  tmp::remove_cr_t<T>>;

}  // namespace internal
}  // namespace wave

#define WAVE_GEOMETRY_REFSELECTOR_HPP

#endif  // WAVE_GEOMETRY_REFSELECTOR_HPP
