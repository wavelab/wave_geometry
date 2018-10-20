/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_SCALARREF_HPP
#define WAVE_GEOMETRY_SCALARREF_HPP

namespace wave {

/** A unary expression wrapping a reference to a scalar
 *
 * Used by the library to wrap a plain scalar (such as double) used in an expression
 * (e.g., scaling a vector).
 *
 * @tparam ScalarType e.g. double
 */
template <typename ScalarType>
struct ScalarRef : public ScalarBase<ScalarRef<ScalarType>> {
    TICK_TRAIT_CHECK(internal::is_scalar<ScalarType>);

    using RhsDerived = tmp::remove_cr_t<ScalarType>;

 public:
    ScalarRef() = delete;
    ScalarRef(const ScalarRef &) noexcept = default;
    ScalarRef(ScalarRef &&) noexcept = default;

    /** Constructs from a reference to scalar */
    explicit ScalarRef(const ScalarType &s) : rhs_{s} {}

    /** Returns stored const reference */
    const ScalarType &rhs() const noexcept {
        return rhs_;
    }

 private:
    const ScalarType &rhs_;
};

namespace internal {

template <typename ScalarType>
struct traits<ScalarRef<ScalarType>> : unary_traits_base<ScalarRef<ScalarType>>,
                                       frameable_vector_traits {
    template <typename NewScalarType>
    using rebind = ScalarRef<NewScalarType>;

    using PlainType = ::wave::Scalar<ScalarType>;
    using Scalar = ScalarType;
    static constexpr int Size = 1;
    enum : int { TangentSize = 1 };
};

// Convert to a leaf expression during evaluation
template <typename ScalarType>
auto evalImpl(expr<ScalarRef>, ScalarType &&s) -> Scalar<ScalarType> {
    return Scalar<ScalarType>{std::forward<ScalarType>(s)};
}

// Implements identity Jacobian
template <typename ScalarType>
auto jacobianImpl(expr<ScalarRef>, const Scalar<ScalarType> &, const ScalarType &) {
    return identity_t<ScalarType>{};
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_SCALARREF_HPP
