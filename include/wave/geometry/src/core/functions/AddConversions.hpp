/**
 * @file
 */

#ifndef WAVE_GEOMETRY_ADDCONVERSIONS_HPP
#define WAVE_GEOMETRY_ADDCONVERSIONS_HPP

namespace wave {
namespace internal {

/** Choose the first directly evaluable leaf type, using either the given candidate or the
 * core leaf types for its operand.
 * For a `Derived` expression of the form `Unary<Rhs>`, the order checked is:
 *
 * 1. `Unary<RhsFolded>` (no conversion)
 * 2. For each type T in the type list `Derived::ConvertTo`:
 *      `Unary<Convert<T, RhsFolded>>`
 *
 * Currently, the fist acceptable conversion is used; in future, the lowest-cost
 * conversion might be used.
 */
template <typename Derived,
          typename Tag,
          template <typename>
          class Rebind,
          typename Rhs,
          typename... ConvertTo>
struct first_directly_evaluable_conversion_unary;


/** Choose the first directly evaluable leaf type, using either the given candidate or the
 * tuple-like list of core leaf types for its operand. This partial specialization is
 * here to expand the list of ConvertTo.
 */
template <typename Derived,
          typename Tag,
          template <typename>
          class Rebind,
          typename Rhs,
          template <typename...>
          class List,
          typename... ConvertTo>
struct first_directly_evaluable_conversion_unary<Derived,
                                                 Tag,
                                                 Rebind,
                                                 Rhs,
                                                 List<ConvertTo...>> {
    // todo: factor out to short-circuit in this most common case
    struct is_evaluable_test : is_directly_evaluable_unary<Tag, eval_t<Rhs>> {
        using type = Derived;
    };

    template <typename ToRhs>
    struct is_evaluable_after_conversion_test
        : tmp::conjunction<is_directly_evaluable_unary<expr<Convert, ToRhs>, eval_t<Rhs>>,
                           is_directly_evaluable_unary<Tag, ToRhs>> {
        using type = Rebind<Convert<ToRhs, Rhs> &&>;
    };

    template <typename T>
    struct nothing_matches {
        using FailedLookup = decltype(evalImpl(Tag{}, std::declval<eval_t<Rhs>>()));
        static_assert(tmp::alwaysFalse<T>(),
                      "Could not find conversions to an applicable evalImpl() function");
    };

    using type =
      typename tmp::disjunction<is_evaluable_test,
                                is_evaluable_after_conversion_test<ConvertTo>...,
                                nothing_matches<void>>::type;
};


/** Choose the first directly evaluable converted type, using either the given
 * binary expression or the tuple-like list of core leaf types for its operands.
 *
 * For a `Derived` expression of the form `Binary<Lhs, Rhs>`, the order checked is:
 *
 * 1. `Binary<Lhs, Rhs>` (no conversion)
 * 2. For each type R in RhsConvertTo:
 *      `Binary<Lhs, Convert<R, Rhs>>`
 * 3. For each type L in LhsConvertTo:
 *      `Binary<Convert<L, Lhs>, Rhs>`
 * 4. For the Cartesian product of the two ConvertTo lists:
 *      `Binary<Convert<L, Lhs>, Convert<R, Rhs>>`
 */
template <typename Derived,
          typename Tag,
          template <typename, typename>
          class Rebind,
          typename Lhs,
          typename Rhs,
          typename LhsConvertTo,
          typename RhsConvertTo>
struct first_directly_evaluable_conversion_binary;

/** Choose the first directly evaluable converted type, using either the given binary
 * expression or the tuple-like list of core leaf types for its operands. This partial
 * specialization is here to expand the list of ConvertTo.
 */
template <typename Derived,
          typename Tag,
          template <typename, typename>
          class Rebind,
          typename Lhs,
          typename Rhs,
          template <typename...>
          class List,
          typename... LhsConvertTo,
          typename... RhsConvertTo>
struct first_directly_evaluable_conversion_binary<Derived,
                                                  Tag,
                                                  Rebind,
                                                  Lhs,
                                                  Rhs,
                                                  List<LhsConvertTo...>,
                                                  List<RhsConvertTo...>> {
    // Helper 1-argument templates
    template <typename ToLhs>
    using replace_left = Rebind<ToLhs, Rhs>;

    template <typename ToRhs>
    using replace_right = Rebind<Lhs, ToRhs>;

    // todo: factor out to short-circuit in this most common case
    struct is_evaluable_test
        : is_directly_evaluable_binary<Tag, eval_t<Lhs>, eval_t<Rhs>> {
        using type = Derived;
    };

    template <typename ToLhs>
    struct convert_left_test
        : tmp::conjunction<is_directly_evaluable_unary<expr<Convert, ToLhs>, eval_t<Lhs>>,
                           is_directly_evaluable_binary<Tag, ToLhs, eval_t<Rhs>>> {
        using type = Rebind<Convert<ToLhs, Lhs> &&, Rhs>;
    };

    template <typename ToRhs>
    struct convert_right_test
        : tmp::conjunction<is_directly_evaluable_unary<expr<Convert, ToRhs>, eval_t<Rhs>>,
                           is_directly_evaluable_binary<Tag, eval_t<Lhs>, ToRhs>> {
        using type = Rebind<Lhs, Convert<ToRhs, Rhs> &&>;
    };

    template <typename ToLhs, typename ToRhs>
    struct convert_both_test
        : tmp::conjunction<is_directly_evaluable_unary<expr<Convert, ToLhs>, eval_t<Lhs>>,
                           is_directly_evaluable_unary<expr<Convert, ToRhs>, eval_t<Rhs>>,
                           is_directly_evaluable_binary<Tag, ToLhs, ToRhs>> {
        using type = Rebind<Convert<ToLhs, Lhs> &&, Convert<ToRhs, Rhs> &&>;
    };

    template <typename T>
    struct nothing_matches {
        using FailedLookup = decltype(
          evalImpl(Tag{}, std::declval<eval_t<Lhs>>(), std::declval<eval_t<Rhs>>()));
        static_assert(tmp::alwaysFalse<T>(),
                      "Could not find conversions to an applicable evalImpl() function");
    };

    using type = typename tmp::expanding_disjunction<
      is_evaluable_test,
      convert_right_test<RhsConvertTo>...,
      convert_left_test<LhsConvertTo>...,
      tmp::apply_cartesian_t<convert_both_test,
                             tmp::type_list<LhsConvertTo...>,
                             tmp::type_list<RhsConvertTo...>>,
      nothing_matches<void>>::type;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_ADDCONVERSIONS_HPP
