/**
 * @file
 */

#ifndef WAVE_GEOMETRY_TEMPLATE_HELPERS_HPP
#define WAVE_GEOMETRY_TEMPLATE_HELPERS_HPP

#include <type_traits>
#include <utility>

namespace wave {
namespace tmp {

// Convenience aliases standard in C++17
template <bool B>
using bool_constant = std::integral_constant<bool, B>;

template <typename...>
using void_t = void;

/** Negate true_type or false_type */
template <class B>
using negation = std::integral_constant<bool, !bool(B::value)>;

/** Logical AND with short-circutiting, from C++17
 * (see http://en.cppreference.com/w/cpp/types/conjunction)
 */
template <class...>
struct conjunction : std::true_type {};
template <class B1>
struct conjunction<B1> : B1 {};
template <class B1, class... Bn>
struct conjunction<B1, Bn...>
    : std::conditional_t<bool(B1::value), conjunction<Bn...>, B1> {};

/** Logical OR with short-circutiting, from C++17
 * (see http://en.cppreference.com/w/cpp/types/disjunction)
 */
template <class...>
struct disjunction : std::false_type {};
template <class B1>
struct disjunction<B1> : B1 {};
template <class B1, class... Bn>
struct disjunction<B1, Bn...>
    : std::conditional_t<bool(B1::value), B1, disjunction<Bn...>> {};

// End of std:: backports. Custom metaprogramming helpers follow.

/** Clean a type of const and reference qualifiers, if any */
template <class T>
using remove_cr_t = std::remove_const_t<std::remove_reference_t<T>>;

/** Check if T and U are the same type ignoring const and reference */
template <class T, class U>
using is_same_cr = std::is_same<remove_cr_t<T>, remove_cr_t<U>>;


/** Helper for is_base_of with CRTP-style inheritance */
template <template <typename...> class Tmpl, typename Derived>
using is_crtp_base_of = std::is_base_of<Tmpl<remove_cr_t<Derived>>, remove_cr_t<Derived>>;

/** Deduce whether a type F is callable with the given Args
 *
 * This is an application of SFINAE, described in
 * https://jguegant.github.io/blogs/tech/sfinae-introduction.html
 * */
template <typename F, class... Args>
struct is_callable {
    // The fallback. This function template can always be chosen.
    template <class>
    static std::false_type test(...);

    // This function template will be chosen if F is callable, since it is more specific.
    // Note only the last expression in decltype determines the type (true_type)
    template <class F_>
    static decltype(std::declval<F_>()(std::declval<Args>()...), void(), std::true_type())
    test(int);

    // Evaluate the value by checking which of the `test` templates is chosen
    // Pass an int argument to give preference to the second template
    static constexpr bool value = decltype(test<F>(0))::value;
};

/** Deduce whether a class specialization exists */
template <typename T, class Enable = void>
struct is_complete : std::false_type {};

template <typename T>
struct is_complete<T, std::enable_if_t<(sizeof(T) > 0)>> : std::true_type {};

/** Helps trigger static_assert if a template is instantiated.
 * See https://stackoverflow.com/q/14637356
 */
template <typename T>
inline constexpr bool alwaysFalse() {
    return false;
}

/** Given a boolean constant template and a list of candidates, sets `type` to the first
 * candidate for which bool(Test<Candidate>{}) is true, or `void` if all candidates fail
 * the test.
 *
 * This trait uses std::disjunction; the instantiation of Test is short-circuited.
 */
template <template <typename...> class Test, typename... Candidates>
struct first_satisfies {
    template <class Candidate>
    struct do_test : Test<Candidate> {
        using type = Candidate;
    };

    struct nothing_matches : std::true_type {
        using type = void;
    };

    using type = typename tmp::disjunction<do_test<Candidates>..., nothing_matches>::type;
};

template <template <typename...> class Test, typename... Candidates>
using first_satisfies_t = typename first_satisfies<Test, Candidates...>::type;


/** Given a derived class and a list of class templates, chooses the first which is a
 * base
 */
template <typename Derived, template <typename> class... BasesToCheck>
struct matching_base_impl;

// Recursive step for unary expression template
template <typename Derived,
          template <typename>
          class Tmpl,
          template <typename>
          class Head,
          template <typename>
          class... Tail>
struct matching_base_impl<Tmpl<Derived>, Head, Tail...> {
    using type = std::conditional_t<
      std::is_base_of<Head<Derived>, Derived>::value,
      Head<Tmpl<Derived>>,                                       // if match, use this
      typename matching_base_impl<Tmpl<Derived>, Tail...>::type  // else, recurse
      >;
};

// Recursive step for binary expression template - enforces matching of both sides
template <typename Lhs,
          typename Rhs,
          template <typename, typename>
          class Tmpl,
          template <typename>
          class Head,
          template <typename>
          class... Tail>
struct matching_base_impl<Tmpl<Lhs, Rhs>, Head, Tail...> {
    using type = std::conditional_t<
      std::is_base_of<Head<Lhs>, Lhs>::value && std::is_base_of<Head<Rhs>, Rhs>::value,
      Head<Tmpl<Lhs, Rhs>>,                                       // if match, use this
      typename matching_base_impl<Tmpl<Lhs, Rhs>, Tail...>::type  // else, recurse
      >;
};

// Base case: none of the given classes matched
template <typename Derived>
struct matching_base_impl<Derived> {
    // "void" triggers the static_assert in the caller
    using type = void;
};

/** Given a derived class and a list of class templates, chooses the first which is a
 * base
 */
template <typename Derived, template <typename> class... BasesToCheck>
struct matching_base {
    // This is separate from matching_base_impl so we can check it and trigger this
    // assert
    using type = typename matching_base_impl<Derived, BasesToCheck...>::type;
    static_assert(!std::is_same<type, void>::value, "No matching base found");
};


template <typename Derived, template <typename> class... BasesToCheck>
using matching_base_t = typename matching_base<Derived, BasesToCheck...>::type;

/** Inspects a function's return and argument types */
template <typename>
struct function_traits;

/** Specialization for general function; other specializations derive this */
template <typename Return, typename... Args>
struct function_traits<Return(Args...)> {
    constexpr static int arity = sizeof...(Args);
    using ReturnType = Return;

    template <int I>
    using ArgType = typename std::tuple_element<I, std::tuple<Args...>>::type;

    using ArgTypes = std::tuple<Args...>;
};

/** Specialization for member function */
template <typename Class, typename Return, typename... Args>
struct function_traits<Return (Class::*)(Args...)> : function_traits<Return(Args...)> {};

/** Specialization for const member function */
template <typename Class, typename Return, typename... Args>
struct function_traits<Return (Class::*)(Args...) const>
    : function_traits<Return(Args...)> {};

/** Specialization for bare function pointer */
template <typename Return, typename... Args>
struct function_traits<Return (*)(Args...)> : function_traits<Return(Args...)> {};

/** Helper to get the return type of a function */
template <typename F>
using return_t = typename function_traits<F>::ReturnType;


}  // namespace tmp
}  // namespace wave

#endif  // WAVE_GEOMETRY_TEMPLATE_HELPERS_HPP
