#include "wave/geometry/core.hpp"

/**
 * These tests demonstrate and check the template helpers.
 *
 * The only reason for defining functions is to organize the checks. The checks
 * are done at compile time, and the functions are never called.
 */
namespace wave {
namespace tmp {

void test_make_index_sequence() {
    static_assert(
      std::is_same<index_sequence<0, 1, 2>, typename make_index_sequence<3>::type>{}, "");
    static_assert(std::is_same<index_sequence<5, 6, 7, 8>,
                               typename make_index_sequence<4, 5>::type>{},
                  "");
    static_assert(
      std::is_same<index_sequence<-3, -2>, typename make_index_sequence<2, -3>::type>{},
      "");
};

void test_concat_index_sequence() {
    using A = index_sequence<0, 1, 2>;
    using B = index_sequence<25, 30>;
    using C = index_sequence<-1>;

    static_assert(std::is_same<index_sequence<0, 1, 2, 25, 30, -1>,
                               typename concat_index_sequence<A, B, C>::type>{},
                  "");
    static_assert(
      std::is_same<index_sequence<-1>, typename concat_index_sequence<C>::type>{}, "");
};

void test_sum_sequence() {
    static_assert(0 == sum_sequence<std::index_sequence<>>::value, "");
    static_assert(42 == sum_sequence<std::index_sequence<42>>::value, "");
    static_assert(8 == sum_sequence<std::index_sequence<3, 0, 1, 1, 2, 1>>::value, "");
};

}  // namespace tmp
}  // namespace wave
