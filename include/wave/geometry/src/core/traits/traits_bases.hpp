/**
 * @file
 */

#ifndef WAVE_GEOMETRY_TRAITS_BASES_HPP
#define WAVE_GEOMETRY_TRAITS_BASES_HPP

namespace wave {

namespace internal {

/** Helper traits to be inherited by nullary expression types **/
template <typename>
struct nullary_traits_base;

/** Traits to be inherited by leaf types **/
template <typename T>
struct leaf_traits_base;

/** Traits to be inherited by traits of compound leaf types **/
template <typename Derived, typename... Primitives>
struct compound_leaf_traits_base;

/** Traits to be inherited by binary types **/
template <typename>
struct binary_traits_base;

/** Traits to be inherited by unary types */
template <typename>
struct unary_traits_base;
template <typename, typename>
struct unary_traits_base_tag;

/** Traits for nullary with one auxilliary type template parameter
 * For example, Random<Translationd>
 */
template <template <typename> class Tmpl, typename Wrapped>
struct nullary_traits_base<Tmpl<Wrapped>> : traits<Wrapped> {
 private:
    using Derived = Tmpl<Wrapped>;

 public:
    // A tag for calling evalImpl()
    using Tag = internal::expr<Tmpl, eval_t<Wrapped>>;
    using PreparedType = Derived &;
    using EvalType = eval_t_unary<Tag, eval_t<Wrapped>>;
    using UniqueLeaves = has_unique_leaves_leaf<Derived>;
};

template <typename T>
struct leaf_traits_base {
    using Tag = leaf;
    using PreparedType = const T &;
    using EvalType = T;
    using OutputFunctor = IdentityFunctor;
    using PlainType = T;
    using UniqueLeaves = has_unique_leaves_leaf<T>;
    using ConvertTo = tmp::type_list<>;
};

template <template <typename...> class Tmpl, typename... Ts, typename... Primitives>
struct compound_leaf_traits_base<Tmpl<Ts...>, Primitives...>
    : leaf_traits_base<Tmpl<Ts...>> {
    using Derived = Tmpl<Ts...>;
    using StorageTuple = std::tuple<internal::storage_t<Primitives>...>;
    using ElementTuple = std::tuple<tmp::remove_cr_t<Primitives>...>;
    using Scalar = std::common_type_t<typename traits<Primitives>::Scalar...>;

    template <typename... NewTs>
    using rebind = Tmpl<NewTs...>;

    template <size_t I>
    using StorageType = std::tuple_element_t<I, StorageTuple>;

    template <size_t I>
    using ElementType = std::tuple_element_t<I, ElementTuple>;

    using TangentSizes = std::integer_sequence<int, traits<Primitives>::TangentSize...>;
    using TangentBlocks = std::tuple<typename traits<Primitives>::TangentType...>;

    enum : int {
        CompoundSize = sizeof...(Primitives),
        TangentSize = tmp::sum_sequence<TangentSizes>::value
    };
};


template <template <typename, typename> class Tmpl,
          typename LhsDerived_,
          typename RhsDerived_>
struct binary_traits_base<Tmpl<LhsDerived_, RhsDerived_>> {
    using LhsDerived = tmp::remove_cr_t<LhsDerived_>;
    using RhsDerived = tmp::remove_cr_t<RhsDerived_>;
    using LhsDerivedRef = LhsDerived_;
    using RhsDerivedRef = RhsDerived_;

    /** The type of the derived template instantiated with a different parameter.
     *
     * This pattern is known as "rebind", "policy clone", "meta-function wrapper
     * idiom". Needed for UnaryExpression.
     */
    template <typename NewLhs, typename NewRhs>
    using rebind = Tmpl<NewLhs, NewRhs>;

    /** A tag for this template */
    using Tag = internal::expr<Tmpl>;

 private:
    // Convenience aliases, not part of traits interface
    using This = Tmpl<LhsDerived_, RhsDerived_>;
    using LhsEval = clean_eval_t<LhsDerived>;
    using RhsEval = clean_eval_t<RhsDerived>;
    // Types of the operands after applying their PreparedType (before evaluation)
    using LhsPrepared = typename traits<LhsDerived>::PreparedType;
    using RhsPrepared = typename traits<RhsDerived>::PreparedType;
    // Type of this expression template rebound to the prepared operands
    using AdaptedType = rebind<LhsPrepared, RhsPrepared>;

    // We want our Prepare step to apply one conversion to each operand, if needed.
    // Find the conversions
    using ConvertedType = typename first_directly_evaluable_conversion_binary<
      AdaptedType,
      Tag,
      rebind,
      LhsPrepared,
      RhsPrepared,
      typename traits<LhsEval>::ConvertTo,
      typename traits<RhsEval>::ConvertTo>::type;

    using ConvertedTraits = traits_safe_t<ConvertedType, This, binary_traits_base>;
    using ConvertedLhs = typename ConvertedTraits::LhsDerived;
    using ConvertedRhs = typename ConvertedTraits::RhsDerived;

 public:
    using PreparedType = ConvertedType &&;
    /** The (leaf) result of evaluating PreparedType */
    using EvalType = eval_t_binary<Tag,
                                   typename traits<ConvertedLhs>::EvalType,
                                   typename traits<ConvertedRhs>::EvalType>;

    using OutputFunctor = IdentityFunctor;
    using UniqueLeaves = has_unique_leaves_binary<LhsDerived, RhsDerived>;
};

// Specialization for regular unary expression with one template parameter (such as
// Inverse)
template <template <typename> class Tmpl, typename RhsDerived_>
struct unary_traits_base<Tmpl<RhsDerived_>> {
    using RhsDerived = tmp::remove_cr_t<RhsDerived_>;
    using RhsDerivedRef = RhsDerived_;

    /** The type of the derived template instantiated with a different parameter.
     *
     * This pattern is known as "rebind", "policy clone", "meta-function wrapper
     * idiom". Needed for UnaryExpression.
     */
    template <typename NewRhs>
    using rebind = Tmpl<NewRhs>;

    /** A tag for this template */
    using Tag = internal::expr<Tmpl>;

 private:
    // Convenience aliases, not part of traits interface
    using This = Tmpl<RhsDerived_>;
    using RhsEval = clean_eval_t<RhsDerived>;
    // Type of the Rhs after applying its PreparedType (before evaluation)
    using RhsPrepared = typename traits<RhsDerived>::PreparedType;

    // Type of this expression template rebound to the prepared rhs
    using AdaptedType = rebind<RhsPrepared>;

    // We want our Prepare step to apply one conversion to the Rhs, if needed. Find
    // the conversion
    using ConvertedType = typename first_directly_evaluable_conversion_unary<
      AdaptedType,
      Tag,
      rebind,
      RhsPrepared,
      typename traits<RhsEval>::ConvertTo>::type;

    using ConvertedRhs =
      typename traits_safe_t<ConvertedType, This, unary_traits_base>::RhsDerived;

 public:
    using PreparedType = ConvertedType &&;
    /** The (leaf) result of evaluating PreparedType */
    using EvalType = eval_t_unary<Tag, typename traits<ConvertedRhs>::EvalType>;

    using OutputFunctor = IdentityFunctor;
    using UniqueLeaves = has_unique_leaves_unary<RhsDerived>;
};

// Specialization for unary expression with an extra parameter (such as Convert),
// With a custom tag provided
template <template <typename, typename> class Tmpl,
          typename Aux,
          typename RhsDerived_,
          typename Tag_>
struct unary_traits_base_tag<Tmpl<Aux, RhsDerived_>, Tag_> {
    using RhsDerived = tmp::remove_cr_t<RhsDerived_>;
    using RhsDerivedRef = RhsDerived_;
    using Tag = Tag_;

    template <typename NewRhs>
    using rebind = Tmpl<Aux, NewRhs>;

 private:
    // Convenience aliases, not part of traits interface
    using This = Tmpl<Aux, RhsDerived_>;
    using RhsEval = clean_eval_t<RhsDerived>;
    // Type of the Rhs after applying its PreparedType (before evaluation)
    using RhsPrepared = typename traits<RhsDerived>::PreparedType;

    // Type of this expression template rebound to the prepared rhs
    using AdaptedType = rebind<RhsPrepared>;

    // We want our Prepare step to apply one conversion to the Rhs, if needed. Find
    // the conversion
    using ConvertedType = typename first_directly_evaluable_conversion_unary<
      AdaptedType,
      Tag,
      rebind,
      RhsPrepared,
      typename traits<RhsEval>::ConvertTo>::type;

    using ConvertedRhs =
      typename traits_safe_t<ConvertedType, This, unary_traits_base_tag>::RhsDerived;

 public:
    using PreparedType = ConvertedType &&;
    /** The (leaf) result of evaluating PreparedType */
    using EvalType = eval_t_unary<Tag, typename traits<ConvertedRhs>::EvalType>;

    using OutputFunctor = IdentityFunctor;
    using UniqueLeaves = has_unique_leaves_unary<RhsDerived>;
};

// Specialization for unary expression with an extra parameter (such as Convert)
template <template <typename, typename> class Tmpl, typename Aux, typename RhsDerived_>
struct unary_traits_base<Tmpl<Aux, RhsDerived_>>
    : unary_traits_base_tag<Tmpl<Aux, RhsDerived_>, expr<Tmpl, Aux>> {};

// Used so we can instantiate traits of non-evaluable types
template <>
struct traits<NotImplemented> {
    using Tag = NotImplemented;
    using ConvertTo = tmp::type_list<>;
    using EvalType = NotImplemented;
    using PlainType = NotImplemented;
    using OutputFunctor = IdentityFunctor;
    using LhsDerived = NotImplemented;
    using RhsDerived = NotImplemented;
};

/** Implementation for standard leaf expression (and others using the `leaf` tag) */
template <typename Derived>
auto evalImpl(leaf, Derived &&l) -> Derived && {
    return std::forward<Derived>(l);
}

/** Identity Jacobian for identity expressions using the `leaf` tag */
template <typename T>
auto jacobianImpl(leaf, const T &, const T &) -> identity_t<T> {
    return identity_t<T>{};
}

/** Get index_sequence of tangent sizes from a type list of expression types */
template <typename List>
struct tangent_sizes_impl;

template <typename List>
using tangent_sizes = typename tangent_sizes_impl<List>::type;

template <template <typename...> class List, typename... Ts>
struct tangent_sizes_impl<List<Ts...>> {
    using type = std::integer_sequence<int, traits<Ts>::TangentSize...>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_TRAITS_BASES_HPP
