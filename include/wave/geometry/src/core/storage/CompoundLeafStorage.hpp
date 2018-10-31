/**
 * @file
 */

#ifndef WAVE_GEOMETRY_COMPOUNDLEAFSTORAGE_HPP
#define WAVE_GEOMETRY_COMPOUNDLEAFSTORAGE_HPP

namespace wave {

/** Storage for a leaf composed of a tuple of Primitive leaves */
template <typename Derived, typename... Primitives>
struct CompoundLeafStorage {
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
    CompoundLeafStorage() : storage_() {}

    CompoundLeafStorage(const CompoundLeafStorage &) = default;
    CompoundLeafStorage(CompoundLeafStorage &&) = default;
    CompoundLeafStorage &operator=(const CompoundLeafStorage &) = default;
    CompoundLeafStorage &operator=(CompoundLeafStorage &&) = default;

    /** Constructs storage tuple from one argument per primitive */
    template <typename... Args,
              std::enable_if_t<
                (sizeof...(Args) > 1) &&
                  std::conjunction<
                    std::is_constructible<internal::storage_t<Primitives>, Args>...>{},
                bool> = true>
    explicit CompoundLeafStorage(Args &&... args)
        : storage_{internal::storage_t<Primitives>{std::forward<Args>(args)}...} {}

    /** Constructs storage tuple from a single (presumably tuple) argument */
    template <typename Arg,
              std::enable_if_t<!std::is_same<std::decay_t<Arg>, CompoundLeafStorage>{} &&
                                 std::is_constructible<StorageType, Arg>{},
                               bool> = true>
    explicit CompoundLeafStorage(Arg &&arg) : storage_{std::forward<Arg>(arg)} {}

    /** Constructs from another expression */
    template <typename OtherDerived>
    explicit CompoundLeafStorage(const ExpressionBase<OtherDerived> &rhs)
        : CompoundLeafStorage{wave::internal::evaluateTo<Derived>(rhs.derived())} {}

    /** Assigns from another expression */
    template <typename Rhs, typename RhsBase = internal::base_tmpl_t<Derived, Rhs>>
    Derived &operator=(const RhsBase &rhs) {
        this->storage =
          wave::internal::evaluateTo<internal::plain_output_t<Derived>>(rhs.derived())
            .value();
        return *static_cast<Derived *>(this);
    }

 public:
    /** Returns const reference to stored tuple */
    auto value() const & noexcept -> const StorageType & {
        return this->storage_;
    }

    /** Returns reference to stored tuple */
    auto value() & noexcept -> StorageType & {
        return this->storage_;
    }

    /** Returns stored tuple of this */
    auto value() && noexcept -> StorageType {
        return std::move(*this).storage_;
    }

 protected:
    ~CompoundLeafStorage() = default;

 private:
    StorageType storage_;
};

namespace internal {}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_COMPOUNDLEAFSTORAGE_HPP
