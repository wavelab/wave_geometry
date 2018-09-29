/**
 * @file
 * Helpers to identify unary, binary, and leaf expressions
 */

#ifndef WAVE_GEOMETRY_TYPE_TRAITS_HPP
#define WAVE_GEOMETRY_TYPE_TRAITS_HPP

// This files uses SFINAE to deduce whether types contain certain members. See
// https://jguegant.github.io/blogs/tech/sfinae-introduction.html for an explanation. We
// purposely uses the comma operator within decltype(). GCC warns the expression before
// the comma has no effect, but we are relying on its SFINAE side-effects.
// Disable the warning just for this file.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-value"

namespace wave {
namespace internal {

/** Determines whether T inherits from ExpressionBase<T> */
TICK_TRAIT(is_derived_expression) {
    template <class T>
    auto require(T && x)->valid<is_true<std::is_base_of<ExpressionBase<T>, T>>>;
};

/** An expression must have these traits */
TICK_TRAIT(valid_expression_traits) {
    template <class T>
    auto require(T &&)
      ->valid<typename T::Tag,
              typename T::PreparedType,
              typename T::EvalType,
              typename T::OutputFunctor,
              typename T::UniqueLeaves>;
};

// This is a non-tick trait as we check for instance of a template
template <typename>
struct is_type_list : std::false_type {};

template <typename... T>
struct is_type_list<::wave::tmp::type_list<T...>> : std::true_type {};

TICK_TRAIT(valid_leaf_traits, valid_expression_traits<_>) {
    template <class T>
    auto require(T &&)
      ->valid<typename T::Scalar,
              is_true_c<std::is_integral<decltype(T::TangentSize)>{} ||
                        std::is_enum<decltype(T::TangentSize)>{}>,
              typename T::PlainType,
              has_type<typename T::ConvertTo, is_type_list<_>>>;
};

TICK_TRAIT(valid_unary_traits, valid_expression_traits<_>) {
    template <class T>
    auto require(T &&)->valid<typename T::RhsDerived, has_template<T::template rebind>>;
};

TICK_TRAIT(valid_binary_traits, valid_expression_traits<_>) {
    template <class T>
    auto require(T &&)
      ->valid<has_template<T::template rebind>,
              typename T::LhsDerived,
              typename T::RhsDerived>;
};

TICK_TRAIT(has_valid_traits) {
    template <class T>
    auto require(T &&)
      ->valid<is_true<valid_expression_traits<typename ::wave::internal::traits<T>>>>;
};

TICK_TRAIT(has_valid_leaf_traits, has_valid_traits<_>) {
    template <class T>
    auto require(T &&)
      ->valid<is_true<valid_leaf_traits<typename ::wave::internal::traits<T>>>>;
};

TICK_TRAIT(has_valid_unary_traits) {
    template <class T>
    auto require(T &&)
      ->valid<is_true<valid_unary_traits<typename ::wave::internal::traits<T>>>>;
};

TICK_TRAIT(has_valid_binary_traits) {
    template <class T>
    auto require(T &&)
      ->valid<is_true<valid_binary_traits<typename ::wave::internal::traits<T>>>>;
};

TICK_TRAIT(is_expression, is_derived_expression<_>, has_valid_traits<_>) {
    template <class T>
    auto require(T &&)
      ->valid<has_template<T::template BaseTmpl>,
              is_true<std::is_base_of<typename T::template BaseTmpl<T>, T>>>;
};

// Traits of unary, binary, and leaf expressions

TICK_TRAIT(is_leaf_expression, is_derived_expression<_>, has_valid_leaf_traits<_>) {
    template <class T>
    auto require(T && x)->valid<decltype(x.value())>;
};

TICK_TRAIT(is_binary_expression, is_derived_expression<_>, has_valid_binary_traits<_>) {
    template <class T>
    auto require(T && x)->valid<decltype(x.lhs()), decltype(x.rhs())>;
};

// A unary expression has a method rhs(), but it does not have a method lhs() or value().
// Thus if there is a rhs, check that it is not binary or leaf.
TICK_TRAIT(is_unary_expression, is_derived_expression<_>, has_valid_unary_traits<_>) {
    template <class T>
    auto require(T && x)->valid<decltype(x.rhs()), is_false<is_binary_expression<T>>>;
};

/** True if type is a scalar (non-expression arithmetic type)
 * Note a scalar is separate from a size-1 vector.
 */
TICK_TRAIT(is_scalar) {
    template <class T>
    auto require(T && x)
      ->valid<is_true<std::is_same<tmp::remove_cr_t<T>, typename traits<T>::Scalar>>>;
};

//
// Convenience enable_if aliases
//

template <typename Derived, typename T = void>
using enable_if_leaf_t = typename std::enable_if<is_leaf_expression<Derived>{}, T>::type;

template <typename Derived, typename T = void>
using enable_if_binary_t =
  typename std::enable_if<is_binary_expression<Derived>{}, T>::type;

template <typename Derived, typename T = void>
using enable_if_unary_t =
  typename std::enable_if<is_unary_expression<Derived>{}, T>::type;

template <typename Derived, typename T = void>
using enable_if_scalar_t = typename std::enable_if<is_scalar<Derived>{}, T>::type;

template <typename Derived, typename T = void>
using enable_if_leaf_or_scalar_t =
  typename std::enable_if<is_leaf_expression<Derived>{} || is_scalar<Derived>{}, T>::type;

/** Empty tag of an expression template for tag dispatching */
template <template <typename...> class Tmpl, typename... Aux>
struct expr {};

/** Empty tag of a leaf for tag dispatching */
struct leaf {};

/** Get a tag type given an instance of a class template */
template <typename T>
using get_expr_tag_t = typename traits<std::decay_t<T>>::Tag;

/** Helper for template rebind */
template <typename T, typename... Us>
using rebind_t = typename traits<std::decay_t<T>>::template rebind<Us...>;

//
// Convenience trait getters
//

/** Gets leaf result type of an expression's evalImpl() call */
template <typename Derived>
using eval_t = typename traits<Derived>::EvalType;

template <typename Derived>
using clean_eval_t = tmp::remove_cr_t<typename traits<Derived>::EvalType>;

/** Gets user-facing output type
 * That is, get PlainType of EvalType, but does not apply output functor
 */
template <typename Derived>
using plain_eval_t = typename traits<clean_eval_t<Derived>>::PlainType;

/** Gets result of applying OutputFunctor to a leaf type
 */
template <typename Derived, typename Leaf>
using output_t = tmp::remove_cr_t<decltype(
  std::declval<typename traits<Derived>::OutputFunctor>()(std::declval<Leaf>()))>;

/** Gets output type of evaluator of an unmodified expression
 * That is, applies OutputFunctor to EvalType
 */
template <typename Derived>
using eval_output_t = output_t<Derived, eval_t<Derived>>;

/** Gets user-facing output type
 * That is, applies OutputFunctor to PlainType
 */
template <typename Derived>
using plain_output_t = output_t<Derived, plain_eval_t<Derived>>;

/** Gets traits of the user-facing result of an expression */
template <typename Derived>
using eval_traits = traits<plain_output_t<Derived>>;

/** Gets the scalar type of the result of an expression */
template <typename Derived>
using scalar_t = typename eval_traits<Derived>::Scalar;

/** Helper for tangent type of an expression */
template <typename Derived>
using tangent_t =
  plain_output_t<decltype(std::declval<Derived>() - std::declval<Derived>())>;

/** Helper for tangent type of an expression */
template <typename Derived>
using plain_tangent_t =
  plain_eval_t<decltype(std::declval<Derived>() - std::declval<Derived>())>;

/** Helper alias for jacobian of one expression wrt another */
template <typename Derived, typename WrtDerived>
using jacobian_t = Eigen::Matrix<scalar_t<Derived>,
                                 eval_traits<Derived>::TangentSize,
                                 eval_traits<WrtDerived>::TangentSize>;

/** Helper alias for jacobian of expression wrt its lhs */
template <typename Derived>
using ljacobian_t = jacobian_t<Derived, typename traits<Derived>::LhsDerived>;

/** Helper alias for jacobian of expression wrt its rhs */
template <typename Derived>
using rjacobian_t = jacobian_t<Derived, typename traits<Derived>::RhsDerived>;

/** Helper alias for identity jacobian type */
template <typename Derived>
using identity_t = IdentityMatrix<scalar_t<Derived>, eval_traits<Derived>::TangentSize>;

//
// Eval type helpers for incomplete types
//

namespace impl {
template <typename... Args>
inline auto evalOrNotImplemented(Args &&...)
  -> decltype(evalImpl(std::declval<Args>()...));

inline auto evalOrNotImplemented(...) -> NotImplemented;
}  // namespace impl

template <typename Tag, typename FoldedRhs>
using eval_t_unary = decltype(evalImpl(Tag(), std::declval<FoldedRhs>()));

// Avoid recusing inside type traits, and instantiating non-existent traits
template <typename Derived, typename Target, typename RhsEval, typename = void>
struct eval_type_safe {
    using type = NotImplemented;
};

template <typename Derived, typename Target, typename RhsEval>
struct eval_type_safe<Derived,
                      Target,
                      RhsEval,
                      tmp::void_t<typename traits<Target>::EvalType>> {
    using type = typename traits<Target>::EvalType;
};

template <typename Derived, typename RhsEval>
struct eval_type_safe<Derived, Derived, RhsEval, void> {
    using type = RhsEval;
};

template <typename Derived, typename Target, typename RhsEval>
using eval_t_safe = typename eval_type_safe<Derived, Target, RhsEval>::type;

// Avoid recusing inside type traits, and instantiating non-existent traits
template <typename Target, typename This, typename Fallback>
struct traits_safe_impl {
    using type = traits<Target>;
};

template <typename This, typename Fallback>
struct traits_safe_impl<This, This, Fallback> {
    using type = Fallback;
};

template <typename Target, typename This, typename Fallback>
using traits_safe_t = typename traits_safe_impl<Target, This, Fallback>::type;

/** True if eval function defined for the given expression type and folded rhs */
template <typename Tag, typename Rhs>
struct is_directly_evaluable_unary
  : tmp::negation<
      std::is_same<decltype(impl::evalOrNotImplemented(Tag(), std::declval<Rhs>())),
                   NotImplemented>> {};

template <typename Tag, typename FoldedLhs, typename FoldedRhs>
using eval_t_binary =
  decltype(evalImpl(Tag(), std::declval<FoldedLhs>(), std::declval<FoldedRhs>()));

/** True if eval function defined for the given expression type and folded rhs */
template <typename Tag, typename Lhs, typename Rhs>
struct is_directly_evaluable_binary
  : tmp::negation<std::is_same<decltype(impl::evalOrNotImplemented(
                                 Tag(), std::declval<Lhs>(), std::declval<Rhs>())),
                               NotImplemented>> {};

// previous eval_type, needs a complete type
// @todo clean up
/** Helper to get the result type of calling evaluate() on an expression */
template <typename Derived, typename Enable = void>
struct eval_type {
    using type = NotImplemented;
};
/** True if eval function defined for all expressions in the folded tree */
TICK_TRAIT(is_directly_evaluable) {
    template <class T>
    auto require(T &&)->valid<is_false<std::is_same<eval_t<T>, NotImplemented>>>;
};


/** `value` is true if an expression is a true tree with unique types.
 * If `value` is true, `type` is a type_list of the leaf types.
 *
 * This alias requires that traits<Derived> is complete.
 * */
template <typename Derived>
using unique_leaves_t = typename traits<Derived>::UniqueLeaves;

/** Determines whether a leaf expression has unique types.
 *
 * Can be used for an incomplete type Derived.
 *
 * This is trivially true. `type` is a type_list with the Derived type, to be used by the
 * other has_unique_leaves_* traits.
 */
template <typename Derived>
struct has_unique_leaves_leaf : std::true_type {
    using type = tmp::type_list<Derived>;
};

/** Determines whether a unary expression has unique types.
 *
 * Can be used for an incomplete type Derived, as long as traits are complete.
 *
 * This is trivially equivalent to unique_leaves_t<RhsDerived>.
 */
template <typename RhsDerived>
struct has_unique_leaves_unary : unique_leaves_t<RhsDerived> {};

/** Determines whether a binary expression has unique types.
 *
 * Can be used for an incomplete type Derived.
 *
 * This is false if either of the branches has non-unique leaves. Otherwise, we
 * concatenate the type_lists of each branch while checking for a repetition.
 */
template <typename LhsDerived, typename RhsDerived>
struct has_unique_leaves_binary
  : std::conditional_t<unique_leaves_t<LhsDerived>::value &&
                         unique_leaves_t<RhsDerived>::value,
                       tmp::concat_if_unique<typename unique_leaves_t<LhsDerived>::type,
                                             typename unique_leaves_t<RhsDerived>::type>,
                       std::false_type> {};


/** Use the derived class's BaseTmpl. For binary expressions, check that both match. */
template <typename...>
struct base_tmpl;

template <typename... T>
using base_tmpl_t = typename base_tmpl<T...>::type;

template <typename Rhs, typename Derived>
struct base_tmpl<Rhs, Derived> {
    using type = typename tmp::remove_cr_t<Rhs>::template BaseTmpl<Derived>;
};

template <typename Lhs, typename Rhs, typename Derived>
struct base_tmpl<Lhs, Rhs, Derived> {
    using type = typename tmp::remove_cr_t<Lhs>::template BaseTmpl<Derived>;
    static_assert(
      std::is_same<type, typename tmp::remove_cr_t<Rhs>::template BaseTmpl<Derived>>(),
      "Operands have mismatching base types");
};

/** Determines whether two types have the same BaseTmpl ("conceptual" base)
 * This one is SFINAE friendly
 */
template <typename A, typename B>
using same_base_tmpl =
  std::is_same<typename tmp::remove_cr_t<A>::template BaseTmpl<tmp::remove_cr_t<B>>,
               typename tmp::remove_cr_t<B>::template BaseTmpl<tmp::remove_cr_t<B>>>;


/** Determines whether two types have the same BaseTmpl ("conceptual" base)
 * This one can be used with an incomplete type
 * @todo combine the base_tmpl traits
 */
template <typename A, typename B>
using same_base_tmpl_i = std::is_same<base_tmpl_t<A, B>, base_tmpl_t<B, B>>;

/** Determines whether rhs has the BaseTmpl of the lhs' TangentType
 */
template <typename L, typename R>
using rhs_is_tangent_of_lhs = same_base_tmpl_i<typename eval_traits<L>::TangentType, R>;


}  // namespace internal
}  // namespace wave

// Restore the warning disabled at top
#pragma GCC diagnostic pop

#endif  // WAVE_GEOMETRY_TYPE_TRAITS_HPP
