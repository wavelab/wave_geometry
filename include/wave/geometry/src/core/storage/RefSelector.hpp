/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_REFSELECTOR_HPP

namespace wave {
namespace internal {

/** Chooses storage type (reference or value) for expressions.
 *
 * Unlike Eigen, we store even lightweight expressions by reference if the storing
 * expression's template arguments specify a reference.
 *
 * The input T is expected to be the type argument to a unary or binary expression
 * template. It should be a wave_geometry expression or scalar type D with some
 * ref-qualifier. It is usually chosen by arg_selector, below.
 *
 * For most expressions, storage_selector uses the straightforward mapping below.
 * An argument of the form D& is stored by lvalue reference, and of form D by value.
 * Arguments of the form D&& are also stored by value; this form is used only to draw
 * attention to leaves being stored by value.
 *
 * | input T | output type  |
 * |---------|--------------|
 * | Expr    | Expr         |
 * | Expr&   | const Expr&  |
 * | Expr&&  | Expr         |
 *
 * This selector may be specialized for types with unusual storage requirements (though no
 * such type currently exists).
 */
template <typename T>
struct storage_selector {
    using type = T;
};

template <typename T>
struct storage_selector<T &> {
    using type = const T &;
};

template <typename T>
struct storage_selector<T &&> {
    using type = T;
};

template <typename T>
using storage_t = typename storage_selector<T>::type;


/** Chooses storage type (reference or value) for expressions in evaluators.
 *
 * This choice differs from storage_t because expressions being evaluated may have been
 * transformed (e.g., conversions added). The ref-qualifiers of T are ignored; instead,
 * its PreparedType traits matters here. Only expressions which are unchanged after
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
using eval_storage_t =
  std::conditional_t<std::is_rvalue_reference<typename traits<T>::PreparedType>{},
                     std::remove_reference_t<T>,
                     const T &>;


/** Chooses template argument for an expression template when an expression is
 * passed into a function with a forwaring reference.
 *
 * For example, template <typename T> auto inverse(T&& expr) -> Inverse<arg_t<T>>
 *
 * arg_t is meant to be used in a user-facing function,
 * e.g. operator+(lhs, rhs). It should either give an non-ref non-const type or an rvalue
 * reference, to be used later by storage_selector. Its output is usually later used by
 * storage_selector to choose a storage type.

 * The default behaviour is:
 *
 * |    input T     | output type |
 * |----------------|-------------|
 * | (const) Expr&  | Expr&       |
 * |         Expr   | Expr        |
 * | (const) Leaf&  | Leaf&       |
 * |         Leaf   | Leaf&&      |
 *
 * There is currently no difference in how storage_selector treats T and T&&. We use
 * T&& here for leaves to be more obvious that things are not like Eigen.
 *
 * This selector may be specialized for types with unusual storage requirements. For
 * example, proxy types are always stored by value in other expressions, and have a
 * specialization of arg_selector.
 */

template <typename T, typename Enable = void>
struct arg_selector;

template <typename T>
struct arg_selector<T &> {
    using type = std::remove_const_t<T> &;
};

template <typename T>
struct arg_selector<T> {
    using type = std::conditional_t<is_leaf_expression<T>{}, T &&, T>;
};

template <typename T>
using arg_t = typename arg_selector<T>::type;


/** Chooses cache storage type (reference or value) for Eigen matrix expressions in
 * Jacobian evaluators.
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
