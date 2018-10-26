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
struct nary_passthrough {};

template <typename Derived, typename... Primitives>
struct compound_traits_base;

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

    using TangentSizes = std::index_sequence<traits<Primitives>::TangentSize...>;

 private:
    // @todo n-ary conversions
    using ConvertedType = rebind<typename traits<Primitives>::PreparedType...>;

 public:
    using PreparedType = ConvertedType &&;
    using Tag = nary_passthrough<ConvertedType>;
    using EvalType = ConvertedType;
    using OutputFunctor = IdentityFunctor;
    using PlainType = rebind<typename traits<Primitives>::PlainType...>;
    using UniqueLeaves =
      tmp::concat_if_unique_many<typename traits<Primitives>::UniqueLeaves...>;
    using ConvertTo = tmp::type_list<>;
    using Scalar = std::common_type_t<typename traits<Primitives>::Scalar...>;

    enum : bool { StoreByRef = true };
    enum : int {
        CompoundSize = sizeof...(Primitives),
        TangentSize = tmp::sum_sequence<TangentSizes>::value
    };
};

template <typename Derived, typename... Ts>
auto evalImpl(nary_passthrough<Derived>, Ts &&... ts) {
    return Derived{std::forward<Ts>(ts)...};
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_NARYSTORAGE_HPP
