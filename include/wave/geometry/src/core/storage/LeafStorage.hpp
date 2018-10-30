/**
 * @file
 */

#ifndef WAVE_GEOMETRY_LEAFSTORAGE_HPP
#define WAVE_GEOMETRY_LEAFSTORAGE_HPP

namespace wave {

/** Mixin providing storage and constructors to satisfy the leaf expression concept */
template <typename Derived, typename StorageType>
struct LeafStorage {
 protected:
    // Tag used to invoke storage constructor
    struct init_storage {};

 private:
    using ValueType = std::decay_t<StorageType>;
    using RefType =
      std::conditional_t<std::is_array<StorageType>{}, ValueType, ValueType &>;
    using ConstRefType = const std::conditional_t<std::is_array<StorageType>{},
                                                  std::decay_t<const StorageType>,
                                                  const ValueType &>;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Default constructor: leaves value uninitialized */
    LeafStorage() = default;
    /** Copy constructor */
    LeafStorage(const LeafStorage &) = default;
    /** Copy assignment operator */
    LeafStorage &operator=(const LeafStorage &) = default;
    /** Move assignment operator */
    LeafStorage &operator=(LeafStorage &&) = default;

    // @todo add noexcept where possible
    // e.g. Eigen 3.3's Matrix has a nothrow move constructor but Quaternion does not
    LeafStorage(LeafStorage &&) = default;

    /** Construct from another expression */
    template <typename OtherDerived>
    explicit LeafStorage(const ExpressionBase<OtherDerived> &rhs)
        : LeafStorage{wave::internal::evaluateTo<Derived>(rhs.derived())} {}

    /** Assign from another expression */
    template <typename OtherDerived>
    Derived &operator=(const ExpressionBase<OtherDerived> &rhs) {
        storage =
          wave::internal::evaluateTo<internal::plain_output_t<Derived>>(rhs.derived())
            .value();
        return *static_cast<Derived *>(this);
    }

    /** Construct */
    template <typename... Args>
    explicit LeafStorage(init_storage, Args &&... args)
        : storage{std::forward<Args>(args)...} {}

    /** Returns const reference to stored value */
    ConstRefType value() const &noexcept {
        return storage;
    }

    /** Returns reference to stored value */
    RefType value() & noexcept {
        return storage;
    }

    /** Returns stored value of this */
    ValueType value() && noexcept {
        return storage;
    }

 protected:
    StorageType storage;
};

namespace internal {
/** Helper to construct a templated leaf expression given the same template */
template <template <typename> class LeafTmpl, typename ImplType>
auto makeLeaf(ImplType &&arg) {
    return LeafTmpl<tmp::remove_cr_t<ImplType>>{std::forward<ImplType>(arg)};
}

/** Helper to return a copy of the value as a vector
 *
 * Works even if .value() returns a scalar. */
template <typename Leaf, enable_if_leaf_t<tmp::remove_cr_t<Leaf>, bool> = true>
auto valueAsVector(adl, Leaf &&leaf) {
    using Vector = Eigen::Matrix<scalar_t<Leaf>, traits<Leaf>::TangentSize, 1>;
    return Vector{std::move(leaf).value()};
}

}  // namespace internal
}  // namespace wave
#endif  // WAVE_GEOMETRY_LEAFSTORAGE_HPP
