#include "wave/geometry/geometry.hpp"

/**
 * These tests demonstrate and check the list-related template helpers.
 *
 * The only reason for defining functions is to organize the checks. The checks
 * are done at compile time, and the functions are never called.
 */
namespace wave {
namespace tmp {

void test_concat_type_list() {
    using A = type_list<int, double>;
    using B = type_list<bool, int, void>;
    using E = type_list<>;

    static_assert(std::is_same<concat_t<A, B>, type_list<int, double, bool, int, void>>{},
                  "");
    static_assert(std::is_same<concat_t<A, E>, A>{}, "");
    static_assert(std::is_same<concat_t<E, E>, E>{}, "");
};

template <typename...>
struct F;

void test_apply() {
    using A = type_list<int, char>;
    using E = type_list<>;

    static_assert(std::is_same<apply_t<F, E>, F<>>{}, "");
    static_assert(std::is_same<apply_t<F, A>, F<int, char>>{}, "");
    static_assert(std::is_same<apply_t<F, int, bool>, F<int, bool>>{}, "");
    static_assert(std::is_same<apply_t<F, void, A>, F<void, int, char>>{}, "");
    static_assert(std::is_same<apply_t<F, void, bool, A>, F<void, bool, int, char>>{},
                  "");
}

void test_apply_each() {
    using A = type_list<int, char>;
    using E = type_list<>;

    static_assert(std::is_same<apply_each_t<F, E>, type_list<>>{}, "");
    static_assert(std::is_same<apply_each_t<F, A>, type_list<F<int>, F<char>>>{}, "");
    static_assert(
      std::is_same<apply_each_t<F, bool, A>, type_list<F<bool, int>, F<bool, char>>>{},
      "");
    static_assert(std::is_same<apply_each_t<F, void, bool, A>,
                               type_list<F<void, bool, int>, F<void, bool, char>>>{},
                  "");
}

void test_apply_cartesian() {
    using A = type_list<int, double>;
    using B = type_list<bool, int, void>;
    using E = type_list<>;

    static_assert(std::is_same<apply_cartesian_t<F, A, B>,
                               type_list<F<int, bool>,
                                         F<int, int>,
                                         F<int, void>,
                                         F<double, bool>,
                                         F<double, int>,
                                         F<double, void>>>{},
                  "");
    static_assert(std::is_same<apply_cartesian_t<F, A, E>, type_list<>>{}, "");
    static_assert(std::is_same<apply_cartesian_t<F, E, A>, type_list<>>{}, "");
    static_assert(std::is_same<apply_cartesian_t<F, E, E>, type_list<>>{}, "");
};

void test_find() {
    using A = type_list<int, bool, int, void>;
    using E = type_list<>;

    static_assert(find<A, int>::value == 0, "");
    static_assert(find<A, bool>::value == 1, "");
    static_assert(find<A, void>::value == 3, "");
    static_assert(find<A, float>::value == -1, "");
    static_assert(find<E, int>::value == -1, "");
};

void test_concat_if_unique() {
    using A = type_list<int, bool>;
    using B = type_list<float, double>;
    using C = type_list<float, double, bool>;
    using E = type_list<>;


    static_assert(std::is_same<typename concat_if_unique<A, B>::type, concat_t<A, B>>{},
                  "");
    static_assert(concat_if_unique<A, B>{}, "");

    static_assert(std::is_same<typename concat_if_unique<A, C>::type, std::false_type>{},
                  "");
    static_assert(concat_if_unique<A, C>{} == false, "");

    static_assert(std::is_same<typename concat_if_unique<A, E>::type, A>{}, "");
    static_assert(concat_if_unique<A, E>{}, "");

    static_assert(std::is_same<typename concat_if_unique<E, E>::type, E>{}, "");
    static_assert(concat_if_unique<E, E>{}, "");
}

template <typename T>
using test_list_t = typename internal::has_unique_leaves_leaf<T>::type;

void test_concat_if_unique_many() {
    using A = concat_if_unique<test_list_t<int>, test_list_t<bool>>;
    using B = concat_if_unique<test_list_t<float>, test_list_t<double>>;
    using D = concat_if_unique<test_list_t<void *>, test_list_t<char>>;

    static_assert(std::is_same<typename concat_if_unique_many<A, B>::type,
                               type_list<int, bool, float, double>>{},
                  "");
    static_assert(concat_if_unique_many<A, B>{}, "");

    static_assert(std::is_same<typename concat_if_unique_many<A, B, D>::type,
                               type_list<int, bool, float, double, void *, char>>{},
                  "");
    static_assert(concat_if_unique_many<A, B, D>{}, "");

    static_assert(concat_if_unique_many<A, B, D, A>{} == false, "");
};

void test_has_unique_leaves() {
    using wave::internal::unique_leaves_t;

    using Y1 = wave::Translationd;
    using Y2 = wave::Minus<wave::Translationd>;
    using Y3 = wave::Sum<wave::TranslationFd<void, bool, char>,
                         wave::TranslationFd<void, char, bool>>;
    using N1 = wave::Sum<Y1, Y1>;
    using N2 = wave::Sum<Y2, Y1>;
    using N3 = wave::Sum<Y3, wave::TranslationFd<void, bool, char>>;


    static_assert(unique_leaves_t<Y1>{}, "");
    static_assert(unique_leaves_t<Y2>{}, "");
    static_assert(unique_leaves_t<Y3>{}, "");
    static_assert(!unique_leaves_t<N1>{}, "");
    static_assert(!unique_leaves_t<N2>{}, "");
    static_assert(!unique_leaves_t<N3>{}, "");
};

void test_cumulative_index_sequence() {
    static_assert(std::is_same<cumulative_index_sequence<>, std::index_sequence<>>{},
                  "empty in = empty out");
    static_assert(std::is_same<cumulative_index_sequence<42>, std::index_sequence<0>>{},
                  "single size = single zero index");
    static_assert(
      std::is_same<cumulative_index_sequence<2, 3, 42>, std::index_sequence<0, 2, 5>>{},
      "always starts at zero, last input ignored");
    static_assert(std::is_same<cumulative_index_sequence<2, 3, 0, 1, 12>,
                               std::index_sequence<0, 2, 5, 5, 6>>{},
                  "impossible input");
};

void test_accumulate_index_sequence() {
    using SeqA = std::index_sequence<42>;
    using SeqB = std::index_sequence<2, 3, 42>;
    using SeqC = std::index_sequence<2, 3, 0, 1, 12>;

    static_assert(std::is_same<accumulate_index_sequence<std::index_sequence<>>,
                               std::index_sequence<>>{},
                  "empty in = empty out");
    static_assert(std::is_same<accumulate_index_sequence<SeqA>, std::index_sequence<0>>{},
                  "single size = single zero index");
    static_assert(
      std::is_same<accumulate_index_sequence<SeqB>, std::index_sequence<0, 2, 5>>{},
      "always starts at zero, last input ignored");
    static_assert(
      std::is_same<accumulate_index_sequence<SeqC>, std::index_sequence<0, 2, 5, 5, 6>>{},
      "impossible input");
};


void test_integer_sequence_element() {
    using Seq = std::index_sequence<21, 42, 63, 84>;

    static_assert(integer_sequence_element<0, Seq>::value == 21, "");
    static_assert(integer_sequence_element<1, Seq>::value == 42, "");
    static_assert(integer_sequence_element<3, Seq>::value == 84, "");
};

}  // namespace tmp
}  // namespace wave
