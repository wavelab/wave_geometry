/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_TYPE_LIST_HPP
#define WAVE_GEOMETRY_TYPE_LIST_HPP

namespace wave {
namespace tmp {
/** Variadic list of types.
 *
 * @todo replace with optimized 3rd party library such as kvasir list.
 * This is a toy implementation offering few, non-optimized features.
 */
template <typename... T>
struct type_list {};

/** Logical OR with short-circutiting, plus expansion of type lists
 * (see http://en.cppreference.com/w/cpp/types/disjunction)
 */
template <class...>
struct expanding_disjunction : std::false_type {};
template <class... T, class... Bn>
struct expanding_disjunction<type_list<T...>, Bn...>
    : expanding_disjunction<T..., Bn...> {};
template <class B1>
struct expanding_disjunction<B1> : B1 {};
template <class B1, class... Bn>
struct expanding_disjunction<B1, Bn...>
    : std::conditional_t<bool(B1::value), B1, expanding_disjunction<Bn...>> {};


/** Concatenate variadic type lists (of the same kind)
 *
 * @todo replace with optimized 3rd party library such as kvasir.
 * This is a toy implementation offering few, non-optimized features.
 */
template <typename...>
struct concat;

template <typename... T>
using concat_t = typename concat<T...>::type;

template <template <typename...> class List, typename... As>
struct concat<List<As...>> {
    using type = List<As...>;
};

template <template <typename...> class List,
          typename... As,
          typename... Bs,
          typename... Tail>
struct concat<List<As...>, List<Bs...>, Tail...> : concat<List<As..., Bs...>, Tail...> {};


/** Apply a template to the expanded parameter pack of a type list. Up to two additional
 * arguments can be placed before the list.
 *
 * For example, apply<F, type_list<A, B, C>>::type is F<A, B, C>
 *              apply<F, D, E, type_list<A, B, C>>::type is F<D, E, A, B, C>
 *
 * The template may also be used without a list: apply<F, A, B, C> is F<A, B, C>.
 *
 * @todo rename, replace with 3rd party library such as kvasir.
 */
template <template <typename...> class F, typename... T>
struct apply {
    using type = F<T...>;
};

template <template <typename...> class F, typename... T>
using apply_t = typename apply<F, T...>::type;

template <template <typename...> class F,
          template <typename...>
          class List,
          typename... Items>
struct apply<F, List<Items...>> {
    using type = F<Items...>;
};

template <template <typename...> class F,
          typename A,
          template <typename...>
          class List,
          typename... Items>
struct apply<F, A, List<Items...>> {
    using type = F<A, Items...>;
};

template <template <typename...> class F,
          typename A,
          typename B,
          template <typename...>
          class List,
          typename... Items>
struct apply<F, A, B, List<Items...>> {
    using type = F<A, B, Items...>;
};

/** Apply a template to each parameter pack of a type list, and give the resulting list.
 *
 * Up to two additional arguments can be placed before the list.
 *
 * For example,
 *   apply_each<F, type_list<A, B, C>>::type is type_list<F<A>, F<B>, F<C>>
 *   apply_each<F, D, type_list<A, B, C>>::type is type_list<F<D, A>, F<D, B>, F<D, C>>
 *
 * The template may also be used without a list: apply_each<F, A, B, C> is type_list<F<A,
 * B, C>>.
 *
 * @todo rename, replace with 3rd party library such as kvasir.
 */
template <template <typename...> class F, typename... T>
struct apply_each {};

template <template <typename...> class F, typename... T>
using apply_each_t = typename apply_each<F, T...>::type;

template <template <typename...> class F,
          template <typename...>
          class List,
          typename... Items>
struct apply_each<F, List<Items...>> {
    using type = List<F<Items>...>;
};

template <template <typename...> class F,
          typename A,
          template <typename...>
          class List,
          typename... Items>
struct apply_each<F, A, List<Items...>> {
    using type = List<F<A, Items>...>;
};

template <template <typename...> class F,
          typename A,
          typename B,
          template <typename...>
          class List,
          typename... Items>
struct apply_each<F, A, B, List<Items...>> {
    using type = List<F<A, B, Items>...>;
};

/** Given two type lists A, B, apply their cartesian product to a two-parameter template
 * C, and produce as `type` the list List<C<a1, b1>, C<a1, b2> ...>
 *
 * @todo replace with 3rd party library such as kvasir.
 */
template <template <typename...> class C, typename A, typename B>
struct apply_cartesian;

template <template <typename...> class C, typename A, typename B>
using apply_cartesian_t = typename apply_cartesian<C, A, B>::type;

template <template <typename...> class C,
          template <typename...>
          class List,
          typename... As,
          typename... Bs>
struct apply_cartesian<C, List<As...>, List<Bs...>> {
    using type = concat_t<List<>,  // need empty in case next part is empty
                          apply_each_t<C, As, List<Bs...>>...>;
};

/** `value` is the first index of Target in a type list, or -1 if not found */
template <typename List, typename Target, int = 0>
struct find;

// While Head doesn't match Target, iterate through the list increasing I
template <template <typename...> class List,
          typename Head,
          typename... Tail,
          typename Target,
          int I>
struct find<List<Head, Tail...>, Target, I> : find<List<Tail...>, Target, I + 1> {};

// If Head matches Target we are done
template <template <typename...> class List, typename... Tail, typename Target, int I>
struct find<List<Target, Tail...>, Target, I> {
    constexpr static int value = I;
};

// If we get to the end of the list we are done
template <template <typename...> class List, typename Target, int I>
struct find<List<>, Target, I> {
    constexpr static int value = -1;
};

/** Concatenate lists A and B if no items in B appear in A.
 * If no items in B match items in A, `type` is the concatenated list and `value` is true.
 * If a match is found, `type` is `false_type` and `value` is false.
 *
 * A and B are not checked for duplicates within themselves.
 */
template <typename A, typename B>
struct concat_if_unique;

template <template <typename...> class List, typename... As, typename B0, typename... Bs>
struct concat_if_unique<List<As...>, List<B0, Bs...>>
    : std::conditional_t<find<List<As...>, B0>::value >= 0,
                         std::false_type,
                         concat_if_unique<List<As..., B0>, List<Bs...>>> {};

template <template <typename...> class List, typename... As>
struct concat_if_unique<List<As...>, List<>> : std::true_type {
    using type = List<As...>;
};


/** Concatenate all "unique leaves" lists if no items are common between them.
 *
 * If no items are common in any two lists, `type` is the concatenated list and `value` is
 * true. Otherwise, `type` is `false_type` and `value` is false.
 *
 * Lists are not checked for duplicates within themselves.
 *
 * @todo use better 3rd party metaprogramming library
 *
 * @tparam Us types containing a type list `type` and a boolean `value`
 */
template <typename... Us>
struct concat_if_unique_many : std::true_type {};

// Base case 1: only one list is given
template <typename U1>
struct concat_if_unique_many<U1> : U1 {};

// Base case 2: last two lists
template <typename U1, typename U2>
struct concat_if_unique_many<U1, U2>
    : std::conditional_t<U1::value && U2::value,
                         concat_if_unique<typename U1::type, typename U2::type>,
                         std::false_type> {};

// More than two lists: fold over concat_if_unique or short-circuit if false
// (like std::conjunction)
template <typename U1, typename U2, class... Tail>
struct concat_if_unique_many<U1, U2, Tail...>
    : std::conditional_t<
        bool(concat_if_unique<typename U1::type, typename U2::type>::value),
        concat_if_unique_many<concat_if_unique<typename U1::type, typename U2::type>,
                              Tail...>,
        concat_if_unique<typename U1::type, typename U2::type>> {};


/** Zip two parameter packs of equal length
 *
 * zip<A, B, C>::with<Q, R, S>
 *    -> type_list<type_list<A, Q>, type_list<B, R>, type_list<C, S>>
 *
 * @tparam Args1
 */
template <class... Ts>
struct zip {
    template <class... Us>
    struct with {
        using type = type_list<type_list<Ts, Us>...>;
    };
};

/** Apply operator+ to tuple (or other get()-able container) elements
 */
template <typename Res, typename L, typename R, size_t... Is>
auto elementwisePlusTo(L &&lhs, const R &&rhs, std::index_sequence<Is...>) -> Res {
    using std::get;
    return Res{(get<Is>(std::forward<L>(lhs)) + get<Is>(std::forward<R>(rhs)))...};
}

/** Apply operator- to tuple (or other get()-able container) elements
 */
template <typename Res, typename L, typename R, size_t... Is>
auto elementwiseMinusTo(L &&lhs, const R &&rhs, std::index_sequence<Is...>) -> Res {
    using std::get;
    return Res{(get<Is>(std::forward<L>(lhs)) - get<Is>(std::forward<R>(rhs)))...};
}


template <typename T, typename SeqOut, typename SeqIn, T PrevI>
struct cumulative_index_sequence_impl;

template <typename T, T... Is, T Last, T PrevI>
struct cumulative_index_sequence_impl<T,
                                      std::integer_sequence<T, Is...>,
                                      std::integer_sequence<T, Last>,
                                      PrevI> {
    using type = std::integer_sequence<T, Is...>;
};

template <typename T, T... Is, T NextSize, T... Sizes, T PrevI>
struct cumulative_index_sequence_impl<T,
                                      std::integer_sequence<T, Is...>,
                                      std::integer_sequence<T, NextSize, Sizes...>,
                                      PrevI>
    : cumulative_index_sequence_impl<T,
                                     std::integer_sequence<T, Is..., PrevI + NextSize>,
                                     std::integer_sequence<T, Sizes...>,
                                     PrevI + NextSize> {};


/** Given input object lengths, get a sequence of indices to the objects.
 * For example, for an input {2, 3, 4}, the output would be {0, 2, 5}.
 * Note the last input is ignored, because indexing starts at zero.
 */
template <std::size_t... Is>
using cumulative_index_sequence =
  typename cumulative_index_sequence_impl<std::size_t,
                                          std::index_sequence<>,
                                          std::index_sequence<0, Is...>,
                                          0>::type;


template <typename Seq>
struct accumulate_index_sequence_impl;

template <typename T, T... Is>
struct accumulate_index_sequence_impl<std::integer_sequence<T, Is...>>
    : cumulative_index_sequence_impl<T,
                                     std::integer_sequence<T>,
                                     std::integer_sequence<T, 0, Is...>,
                                     0> {};
/** Given input sequence object lengths, get a sequence of indices to the objects.
 * For example, for an input {2, 3, 4}, the output would be {0, 2, 5}.
 * Note the last input is ignored, because indexing starts at zero.
 */
template <typename Seq>
using accumulate_index_sequence = typename accumulate_index_sequence_impl<Seq>::type;


/** Get I-th integer from an integer sequence as `value` */
template <size_t I, typename Seq>
struct integer_sequence_element;

template <typename T, T Head, T... Tail>
struct integer_sequence_element<0, std::integer_sequence<T, Head, Tail...>>
    : std::integral_constant<T, Head> {};

template <size_t I, typename T, T Head, T... Tail>
struct integer_sequence_element<I, std::integer_sequence<T, Head, Tail...>>
    : integer_sequence_element<I - 1, std::integer_sequence<T, Tail...>> {};


/** Filters a type list using a predicate.
 *
 * For example, if P<A>::value == P<C>::value = true  and P<B>::value == false,
 *
 *   filter<P, type_list<A, B, C>>::type is type_list<A, C>
 */
template <template <typename> class P, typename List>
struct filter;

template <template <typename> class P, typename List>
using filter_t = typename filter<P, List>::type;

// Use concat to do the work
template <template <typename> class P, template <typename...> class List, typename... Ts>
struct filter<P, List<Ts...>> {
    using type = concat_t<std::conditional_t<P<Ts>::value, List<Ts>, List<>>...>;
};

// Trivial specialization for empty list
template <template <typename> class P, template <typename...> class List>
struct filter<P, List<>> {
    using type = List<>;
};


}  // namespace tmp
}  // namespace wave

#endif  // WAVE_GEOMETRY_TYPE_LIST_HPP
