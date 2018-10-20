/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_REFSELECTOR_HPP

namespace wave {
namespace internal {
/** Chooses storage type (reference or value) for expressions.
 *
 * The input T is expected to be the type parameter to a unary or binary expression
 * template. It should be a wave_geometry expression or scalar type D with some ref
 * qualifier. It is usually chosen by arg_t, below.
 *
 * If T is an D&& (denoting an expression passed as a temporary), we want to
 * store it by value. If T is D& we store it by lvalue reference. If T is clean (denoting
 * an expression passed by lvalue reference), we store it according to its StoreByRef
 * trait.
 *
 * Unlike Eigen, we store even lightweight expressions by reference (unless they set
 * StoreByRef false (indicated by Prox below).
 *
 * | input   | output       |
 * |---------|--------------|
 * | Expr    | const Expr&  |
 * | Expr&   | const Expr&  |
 * | Expr&&  | Expr         |
 * | Prox    | const Prox&  |
 * | Prox&   | Prox         |
 * | Prox&&  | Prox         |
 */
template <typename T>
using ref_sel_t =
  std::conditional_t<std::is_rvalue_reference<T>{} ||
                       !(std::is_lvalue_reference<T>{} || traits<T>::StoreByRef),
                     std::remove_reference_t<T>,
                     const std::remove_reference_t<T> &>;

/** Chooses storage type (reference or value) for expressions in evaluators.
 *
 * This choice differs from ref_sel_t because expressions being evaluated may have been
 * transformed (e.g., conversions added). Only expressions which are unchanged after
 * transformation, including leaves, are stored by reference.
 *
 * | input  | output      |
 * |--------|-------------|
 * | Expr   | Expr        |
 * | Expr&& | Expr        |
 * | Leaf   | const Leaf& |
 * | Leaf&& | Leaf        |
 */
template <typename T>
using eval_ref_sel_t =
  std::conditional_t<std::is_rvalue_reference<typename traits<T>::PreparedType>{},
                     std::remove_reference_t<T>,
                     const T &>;

/** Chooses template argument for an expression template when an expression is
 * passed into a function with a forwaring reference.
 *
 * For example, template <typename T> auto inverse(T&& expr) -> Inverse<arg_t<T>>
 *
 * | input  | output      |
 * |--------|-------------|
 * | Expr&  | const Expr& |
 * | Expr   | Expr&&      |
 *
 * It has the same goal as ref_sel_t, but is meant to be used in a user-facing function,
 * e.g. operator+(lhs, rhs). It should either give an non-ref non-const type or an rvalue
 * reference, to be used later by ref_sel_t.
 */
template <typename T>
using arg_t = std::conditional_t<std::is_reference<T>{}, tmp::remove_cr_t<T>, T &&>;

/** Chooses cache storage type (reference or value) for Jacobian evaluators.
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
  std::conditional_t<std::is_lvalue_reference<T>{},
                     typename Eigen::internal::ref_selector<tmp::remove_cr_t<T>>::type,
                     const tmp::remove_cr_t<T>>;

}  // namespace internal
}  // namespace wave

#define WAVE_GEOMETRY_REFSELECTOR_HPP

#endif  // WAVE_GEOMETRY_REFSELECTOR_HPP
