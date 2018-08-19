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
          template <typename...> class List,
          typename... Items>
struct apply<F, List<Items...>> {
    using type = F<Items...>;
};

template <template <typename...> class F,
          typename A,
          template <typename...> class List,
          typename... Items>
struct apply<F, A, List<Items...>> {
    using type = F<A, Items...>;
};

template <template <typename...> class F,
          typename A,
          typename B,
          template <typename...> class List,
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
 *   apply<C, D, type_list<A, B, C>>::type is type_list<F<D, A>, F<D, B>, F<D, C>>
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
          template <typename...> class List,
          typename... Items>
struct apply_each<F, List<Items...>> {
    using type = List<F<Items>...>;
};

template <template <typename...> class F,
          typename A,
          template <typename...> class List,
          typename... Items>
struct apply_each<F, A, List<Items...>> {
    using type = List<F<A, Items>...>;
};

template <template <typename...> class F,
          typename A,
          typename B,
          template <typename...> class List,
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
          template <typename...> class List,
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


}  // namespace tmp
}  // namespace wave

#endif  // WAVE_GEOMETRY_TYPE_LIST_HPP
