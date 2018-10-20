/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_SCALAR_HPP
#define WAVE_GEOMETRY_SCALAR_HPP

namespace wave {

/** A leaf expression wrapping a scalar
 *
 * @tparam ScalarType_ e.g. double
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

namespace internal {
template <typename ScalarType>
struct traits<Scalar<ScalarType>> : leaf_traits_base<Scalar<ScalarType>>,
                                    frameable_vector_traits {
    template <typename NewScalarType>
    using rebind = ::wave::Scalar<NewScalarType>;

    using PlainType = ::wave::Scalar<ScalarType>;
    using Scalar = ScalarType;
    static constexpr int Size = 1;
    enum : int { TangentSize = 1 };
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_SCALAR_HPP
