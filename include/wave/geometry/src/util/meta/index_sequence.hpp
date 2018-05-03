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
 */
template <int... Indices>
struct index_sequence {
    using type = index_sequence<Indices...>;
};

/** Constructs an index_sequence<S, S+1, ..., S+N-1> */
template <int N, int S = 0, int...>
struct make_index_sequence;

/** Concatenates any number of index sequences into one */
template <typename... Seqs>
struct concat_index_sequence;

// Generate an index_sequence<S, ..., S + N - 1> via recursion
// Count down in the first argument while adding one number to the start of the
// sequence.
template <int N, int S, int... Indices>
struct make_index_sequence : make_index_sequence<N - 1, S, S + N - 1, Indices...> {};

// Base case: the counter reached S.
// Remove the counter, leaving only the finished sequence.
template <int S, int... Indices>
struct make_index_sequence<0, S, Indices...> : index_sequence<Indices...> {};

// Recursively combine any number of index sequences
// Concatenate two at a time. Always append the indices from the second sequence
// onto the first, and discard the second sequence.
template <int... I1, int... I2, typename... Seqs>
struct concat_index_sequence<index_sequence<I1...>, index_sequence<I2...>, Seqs...>
  : concat_index_sequence<index_sequence<I1..., I2...>, Seqs...> {};

// Base case: one sequence left
template <int... I1>
struct concat_index_sequence<index_sequence<I1...>> {
    using type = index_sequence<I1...>;
};

}  // namespace tmp
}  // namespace wave

#endif  // WAVE_GEOMETRY_INDEX_SEQUENCE_HPP
