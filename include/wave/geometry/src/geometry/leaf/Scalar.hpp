/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_SCALAR_HPP
#define WAVE_GEOMETRY_SCALAR_HPP

namespace wave {

/** A leaf expression wrapping a scalar
 *
 * @tparam ScalarType e.g. double
 * Specialization for storage by value
 */
template <typename ScalarType>
class Scalar : public ScalarBase<Scalar<ScalarType>> {
    using StorageType = ScalarType;
    TICK_TRAIT_CHECK(internal::is_scalar<ScalarType>);

 public:
    Scalar() = default;

    /** Constructs from a scalar
     *
     * Purposely not explicit, allowing
     * Scalar<double> s = 3.0;
     */
    Scalar(StorageType s) : storage{s} {}

    /** Assigns a scalar value */
    Scalar &operator=(StorageType s) {
        this->storage = s;
        return *this;
    }

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(Scalar)

    /** Returns const reference to stored value */
    const StorageType &value() const &noexcept {
        return storage;
    }

    /** Returns reference to stored value */
    StorageType &value() & noexcept {
        return storage;
    }

    /** Returns stored value */
    StorageType value() && noexcept {
        return storage;
    }

 private:
    StorageType storage;
};

/** A unary expression wrapping a reference to a scalar
 *
 * @tparam ScalarType_ e.g. double&
 * Specialization for storage by value
 */
template <typename ScalarType>
class Scalar<ScalarType &> : public ScalarBase<Scalar<ScalarType &>> {
 public:
    Scalar() = delete;
    Scalar(const Scalar &) noexcept = default;
    Scalar(Scalar &&) noexcept = default;

    /** Constructs from a reference to scalar */
    explicit Scalar(const ScalarType &s) : rhs_{s} {}

    /** Returns stored const reference */
    const ScalarType &rhs() const noexcept {
        return rhs_;
    }

 private:
    const ScalarType &rhs_;
};


namespace internal {
template <typename ScalarType>
struct traits<Scalar<ScalarType>> : leaf_traits_base<Scalar<ScalarType>>,
                                    frameable_vector_traits {
    template <typename NewScalarType>
    using rebind = ::wave::Scalar<NewScalarType>;

    using PlainType = ::wave::Scalar<ScalarType>;
    using Scalar = tmp::remove_cr_t<ScalarType>;
    static constexpr int Size = 1;
    enum : int { TangentSize = 1 };
};

template <typename ScalarType>
struct traits<Scalar<ScalarType &>> : unary_traits_base<Scalar<ScalarType &>>,
                                      frameable_vector_traits {
    template <typename NewScalarType>
    using rebind = ::wave::Scalar<NewScalarType>;

    using PlainType = ::wave::Scalar<ScalarType>;
    using Scalar = tmp::remove_cr_t<ScalarType>;
    static constexpr int Size = 1;
    enum : int { TangentSize = 1 };
};

// Convert Scalar<ScalarType&> to a leaf expression during evaluation
template <typename ScalarType>
auto evalImpl(expr<Scalar>, const ScalarType &s) -> Scalar<ScalarType> {
    return Scalar<ScalarType>{s};
}

// Implements identity Jacobian
template <typename ScalarType>
auto jacobianImpl(expr<Scalar>, const Scalar<ScalarType> &, const ScalarType &) {
    return identity_t<ScalarType>{};
}


}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_SCALAR_HPP
