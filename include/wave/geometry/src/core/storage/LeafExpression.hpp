/**
 * @file
 */

#ifndef WAVE_GEOMETRY_LEAFEXPRESSION_HPP
#define WAVE_GEOMETRY_LEAFEXPRESSION_HPP

namespace wave {

/** Mixin providing storage and constructors to satisfy the LeafExpression concept */
template <typename StorageType, typename Derived>
struct LeafExpression {
 protected:
    struct init_storage {};


    using ValueType = tmp::decay_t<StorageType>;
    using RefType =
      tmp::conditional_t<std::is_array<StorageType>{}, ValueType, ValueType &>;
    using ConstRefType = const tmp::conditional_t<std::is_array<StorageType>{},
                                                  tmp::decay_t<const StorageType>,
                                                  const ValueType &>;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /** Default constructor: leaves value uninitialized */
    LeafExpression() = default;
    /** Copy constructor */
    LeafExpression(const LeafExpression &) = default;
    /** Copy assignment operator */
    LeafExpression &operator=(const LeafExpression &) = default;
    /** Move assignment operator */
    LeafExpression &operator=(LeafExpression &&) = default;

    // @todo add noexcept where possible
    // e.g. Eigen 3.3's Matrix has a nothrow move constructor but Quaternion does not
    LeafExpression(LeafExpression &&) = default;

    /** Construct from another expression */
    template <typename OtherDerived>
    explicit LeafExpression(const ExpressionBase<OtherDerived> &rhs)
        : LeafExpression{wave::internal::evaluateTo<Derived>(rhs.derived())} {}

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
    explicit LeafExpression(init_storage, Args &&... args)
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
auto makeLeaf(ImplType &&arg) -> LeafTmpl<tmp::remove_cr_t<ImplType>> {
    return LeafTmpl<tmp::remove_cr_t<ImplType>>{std::forward<ImplType>(arg)};
}
}  // namespace internal
}  // namespace wave
#endif  // WAVE_GEOMETRY_LEAFEXPRESSION_HPP
