/**
 * @file
 */

#ifndef WAVE_GEOMETRY_NARYSTORAGE_HPP
#define WAVE_GEOMETRY_NARYSTORAGE_HPP

namespace wave {

/** Mixin providing storage for an n-ary expression */
template <typename Derived, typename... Primitives>
struct NaryStorage {
 public:
    using StorageType = std::tuple<internal::storage_t<Primitives>...>;

    template <typename Dummy>
    struct helper {
        static constexpr bool all_default_constructible() {
            return tmp::conjunction<
              std::is_default_constructible<internal::storage_t<Primitives>>...>{};
        }

        static constexpr bool all_leaves() {
            return tmp::conjunction<internal::is_leaf_expression<Primitives>...>{};
        }

        static constexpr bool all_vector_leaves() {
            return tmp::conjunction<internal::is_vector_leaf<Primitives>...>{};
        }
    };

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Default-constructs compound storage only if all elements are default-constructible
     */
    template <typename Dummy = void,
              std::enable_if_t<helper<Dummy>::all_default_constructible(), bool> = true>
    NaryStorage() : storage_() {}

    NaryStorage(const NaryStorage &) = default;
    NaryStorage(NaryStorage &&) = default;
    NaryStorage &operator=(const NaryStorage &) = default;
    NaryStorage &operator=(NaryStorage &&) = default;

    template <typename... Args, std::enable_if_t<(sizeof...(Args) > 1), bool> = true>
    explicit NaryStorage(Args &&... args) : storage_{std::forward<Args>(args)...} {}

    /** Construct from another expression - enabled if we are a compound of leaves */
    template <typename OtherDerived,
              std::enable_if_t<helper<OtherDerived>::all_leaves(), bool> = true>
    explicit NaryStorage(const ExpressionBase<OtherDerived> &rhs)
        : NaryStorage{wave::internal::evaluateTo<Derived>(rhs.derived())} {}

    /** Assign from another n-ary expression */
    template <typename Rhs, typename RhsBase = internal::base_tmpl_t<Derived, Rhs>>
    Derived &operator=(const RhsBase &rhs) {
        this->storage =
          wave::internal::evaluateTo<internal::plain_output_t<Derived>>(rhs.derived())
            .value();
        return *static_cast<Derived *>(this);
    }

 public:
    template <size_t I>
    auto const &get() const &noexcept {
        return std::get<I>(this->storage_);
    }

    template <size_t I>
      auto &get() & noexcept {
        return std::get<I>(this->storage_);
    }

    template <size_t I>
      auto &&get() && noexcept {
        return std::get<I>(std::move(*this).storage_);
    }

 protected:
    ~NaryStorage() = default;

 private:
    StorageType storage_;
};

namespace internal {

template <typename Derived>
struct nary_eval {};

template <typename Derived, typename... Primitives>
struct compound_traits_base;

template <typename Derived, typename... Primitives>
struct compound_vector_traits_base;

template <template <typename...> class Tmpl, typename... Ts, typename... Primitives>
struct compound_traits_base<Tmpl<Ts...>, Primitives...> {
    using Derived = Tmpl<Ts...>;
    using StorageTuple = std::tuple<internal::storage_t<Primitives>...>;
    using ElementTuple = std::tuple<tmp::remove_cr_t<Primitives>...>;

    template <typename... NewTs>
    using rebind = Tmpl<NewTs...>;

    template <size_t I>
    using StorageType = std::tuple_element_t<I, StorageTuple>;

    template <size_t I>
    using ElementType = std::tuple_element_t<I, ElementTuple>;

    using TangentBlocks = std::tuple<typename eval_traits<Primitives>::TangentType...>;

 private:
    // @todo n-ary conversions
    using ConvertedType = rebind<typename traits<Primitives>::PreparedType...>;

 public:
    using PreparedType = ConvertedType &&;
    using EvalType = rebind<typename traits<Primitives>::EvalType...>;
    using Tag = nary_eval<EvalType>;
    using OutputFunctor = IdentityFunctor;
    using PlainType = rebind<plain_output_t<Primitives>...>;
    using UniqueLeaves =
      tmp::concat_if_unique_many<typename traits<Primitives>::UniqueLeaves...>;
    using ConvertTo = tmp::type_list<>;
    using Scalar = common_scalar_t<Primitives...>;

    enum : bool { StoreByRef = true };
    enum : int {
        CompoundSize = sizeof...(Primitives),
        Size =
          tmp::sum_sequence<std::index_sequence<eval_traits<Primitives>::Size...>>::value,
        TangentSize = tmp::sum_sequence<
          std::index_sequence<eval_traits<Primitives>::TangentSize...>>::value
    };
};

template <template <typename...> class Tmpl, typename... Ts, typename... Ps>
struct compound_vector_traits_base<Tmpl<Ts...>, Ps...>
    : compound_traits_base<Tmpl<Ts...>, Ps...> {
 private:
    using TraitsBase = compound_traits_base<Tmpl<Ts...>, Ps...>;

 public:
    using TangentType = typename TraitsBase::PlainType;
};


template <typename Derived, typename... Ts>
auto evalImpl(nary_eval<Derived>, Ts &&... ts) {
    return Derived{std::forward<Ts>(ts)...};
}

template <template <typename...> class Expr, typename... Args>
auto makeCompound(Args &&... args) {
    return Expr<arg_t<Args>...>{std::forward<Args>(args)...};
}

template <typename OtherExpr, typename... Args>
auto makeCompoundLike(Args &&... args) {
    return typename traits<OtherExpr>::template rebind<tmp::remove_cr_t<Args>...>{
      std::forward<Args>(args)...};
}

// Helper for valueAsVector below
template <typename Derived, std::size_t... Is>
auto naryValueAsVectorExpand(Derived &&nary, std::index_sequence<Is...>) {
    using EmptyVec = Eigen::Matrix<scalar_t<Derived>, 0, 1>;
    using Vector = Eigen::Matrix<scalar_t<Derived>, traits<Derived>::TangentSize, 1>;
    auto vec = Vector{};

    // Use Eigen::CommaInitializer as a convenient way of building up the vector, with
    // bounds checking. We want the result to be as if we had written vec << a, b, c...
    // but with items from the n-ary tuple.
    // The only way to construct a CommaInitializer is by giving it something, so start
    // with an empty matrix.
    auto comma_init = vec << EmptyVec{};

    // For each item, pass it to the CommaInitializer using overloaded comma operator
    // Note we need the void() to force calling the non-overloaded comma operator
    int foreach[] = {
      (comma_init, std::forward<Derived>(nary).template get<Is>().value(), void(), 0)...};
    (void) foreach;
    return comma_init.finished();
}

/** Concatenates values of a compound of vectors into a vector
 *
 * @todo make another storage class that's already a block vector, instead of a tuple?
 */
template <typename Derived,
          std::enable_if_t<is_compound_vector_expression<tmp::remove_cr_t<Derived>>{},
                           bool> = true>
auto valueAsVector(adl, Derived &&nary) {
    using Indices = std::make_index_sequence<traits<Derived>::CompoundSize>;
    return naryValueAsVectorExpand(std::forward<Derived>(nary), Indices{});
}

// Helper for compoundIsApprox below
template <typename L, typename R, std::size_t... Is>
auto compoundIsApproxExpand(const ExpressionBase<L> &a,
                            const ExpressionBase<R> &b,
                            const scalar_t<L> prec,
                            std::index_sequence<Is...>) {
    bool res = true;
    int foreach[] = {(res = res && a.derived().template get<Is>().isApprox(
                                     b.derived().template get<Is>(), prec),
                      0)...};
    (void) foreach;
    return res;
}
/** Calls isApprox() for each element of a compound expression, and returns the fold
 * over AND */
template <typename L, typename R>
auto compoundIsApprox(const ExpressionBase<L> &a,
                      const ExpressionBase<R> &b,
                      const scalar_t<L> prec) {
    using Indices = std::make_index_sequence<traits<L>::CompoundSize>;
    return compoundIsApproxExpand(a.derived(), b.derived(), prec, Indices{});
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_NARYSTORAGE_HPP
