/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_INDEX_SEQUENCE_HPP
#define WAVE_GEOMETRY_INDEX_SEQUENCE_HPP

namespace wave {
namespace tmp {

/** We have an integer `N`, known at compiler time, and want the compiler to
 * generate a function call `f(1, ..., N)`. For example, if N is 3, we want to
 * call `f(1, 2, 3)`.
 *
 * The solution involves variadic templates. This template, `index_sequence`,
 * holds the sequence 1..N as template parameters. When we pass an instance of
 * it to a variadic function template, the sequence can become a parameter pack.
 * For example,
 *
 * ```
 * template<int... Is>
 * void callF(index_sequence<Is...> indices) { f(Is...); }
 *
 * int main() {
 *     callF(index_sequence<1, 2, 3>{});
 *     f(1, 2, 3);
 * }
 * ```
 *
 * The call above is equivalent to f(1, 2, 3). Of course, it's not useful if you
 * have to type out index_sequence<1, 2, 3>. That's why we define the template
 * `make_index_sequence`, which lets us call `callF(make_index_sequence<3>{});`.
 *
 * This approach is common enough that `index_sequence` and
 * `make_index_sequence` were added to the standard library in C++14. Since
 * they are not in C++11, we define a simple implementation here.
 *
 * See also:
 *
 * - http://en.cppreference.com/w/cpp/utility/integer_sequence
 * - The "indices trick": http://loungecpp.wikidot.com/tips-and-tricks:indices
 * - Variadic templates:
 * http://eli.thegreenplace.net/2014/variadic-templates-in-c
 *
 *  @note incompatible with std::integer_sequence due to missing type parameter
 *  @todo remove in c++14
 */
template <int... Indices>
struct index_sequence {
    using type = index_sequence<Indices...>;
};

/** Constructs an index_sequence<S, S+1, ..., S+N-1> */
template <int N, int S = 0, int...>
struct make_index_sequence;

// Generate an index_sequence<S, ..., S + N - 1> via recursion
// Count down in the first argument while adding one number to the start of the
// sequence.
template <int N, int S, int... Indices>
struct make_index_sequence : make_index_sequence<N - 1, S, S + N - 1, Indices...> {};

// Base case: the counter reached S.
// Remove the counter, leaving only the finished sequence.
template <int S, int... Indices>
struct make_index_sequence<0, S, Indices...> : index_sequence<Indices...> {};


/** Concatenates any number of integer sequences into one
 *
 * @note works for std::integer_sequence, not tmp::index_sequence*/
template <typename T, typename... Seqs>
struct concat_integer_sequence;

template <typename... Seqs>
using concat_index_sequence = concat_integer_sequence<std::size_t, Seqs...>;

// Recursively combine any number of index sequences
// Concatenate two at a time. Always append the indices from the second sequence
// onto the first, and discard the second sequence.
template <typename T,
          template <typename, T...>
          class Seq,
          T... I1,
          T... I2,
          typename... Tail>
struct concat_integer_sequence<T, Seq<T, I1...>, Seq<T, I2...>, Tail...>
    : concat_integer_sequence<T, Seq<T, I1..., I2...>, Tail...> {};

// Base case: one sequence left
template <typename T, template <typename, T...> class Seq, T... I1>
struct concat_integer_sequence<T, Seq<T, I1...>> {
    using type = Seq<T, I1...>;
};


/** Filters an integer sequence using a predicate.
 *
 * For example, if P<A>::value == P<C>::value = true  and P<B>::value == false, then
 *
 *   filter_integer_sequence<int, P, integer_sequence<int, A, B, C>>::type is
 * integer_sequence<int, A, C>
 */
template <typename T, template <T> class P, typename Seq>
struct filter_integer_sequence;

template <typename T, template <T> class P, typename Seq>
using filter_integer_sequence_t = typename filter_integer_sequence<T, P, Seq>::type;

template <template <std::size_t> class P, typename Seq>
using filter_index_sequence_t =
  typename filter_integer_sequence<std::size_t, P, Seq>::type;


// Use concat to do the work
template <typename T, template <T> class P, template <typename, T...> class Seq, T... Is>
struct filter_integer_sequence<T, P, Seq<T, Is...>> {
    using type = typename concat_integer_sequence<
      T,
      std::conditional_t<P<Is>::value, Seq<T, Is>, Seq<T>>...>::type;
};

// Trivial specialization for empty list
template <typename T, template <T> class P, template <typename, T...> class Seq>
struct filter_integer_sequence<T, P, Seq<T>> {
    using type = Seq<T>;
};

/** Get sum of integer sequence as `value`*/
template <typename Seq>
struct sum_sequence;

template <typename T>
struct sum_sequence<std::integer_sequence<T>> : std::integral_constant<T, 0> {};

template <typename T, T Sum>
struct sum_sequence<std::integer_sequence<T, Sum>> : std::integral_constant<T, Sum> {};

template <typename T, T I1, T I2, T... Tail>
struct sum_sequence<std::integer_sequence<T, I1, I2, Tail...>>
    : sum_sequence<std::integer_sequence<T, I1 + I2, Tail...>> {};

}  // namespace tmp
}  // namespace wave

#endif  // WAVE_GEOMETRY_INDEX_SEQUENCE_HPP
