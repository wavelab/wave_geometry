/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_FACTOR_HPP
#define WAVE_GEOMETRY_FACTOR_HPP

namespace wave {
namespace internal {

template <typename TheFunctorReturns, typename TheMeasurementValueIs>
struct factor_functor_check {
    static_assert(
      internal::same_base_tmpl<TheFunctorReturns, TheMeasurementValueIs>{},
      "Functor must return an expression of the same space as this factor's measurement");
    using type = std::true_type;
};


}  // namespace internal

/**
 * Template for factors of different variable types.
 *
 * The residuals in a least squares function can be described as
 * @f[
 * r(\Theta) = f(\Theta) - Z
 * @f]
 *
 * where
 *
 * - @f$ Z @f$ is the actual measurement
 * - @f$ \Theta @f$ is one or more variables
 * - @f$ f(\Theta) @f$ is the measurement function, producing an estimate of @f$
 * Z @f$ given some variables @f$ \Theta @f$
 *
 * These three things are stored in a Factor. Most of the work in defining a
 * factor is defining a measurement function.
 * See Factor::evaluate for the required function signature.
 *
 * It is typically unnecessary to directly instantiate or derive this template;
 * instead, use FactorGraph::addFactor.
 *
 * @tparam Functor Type of a function object
 * @tparam MeasType The type of measurement used by this factor. Its value type must be in
 * the same space as the output of Functor
 * @tparam LeafTypes The list of leaf expressions which are inputs to Functor
 */
template <typename Functor, typename MeasType, typename... LeafTypes>
class Factor : public FactorBase {
    static constexpr auto NumVars = sizeof...(LeafTypes);
    using ArrayType = std::array<std::shared_ptr<FactorVariableBase>, NumVars>;
    using ResidualType = decltype(MeasType::value);
    using ResidualVectorType = BlockVector<ResidualType>;

    static_assert(internal::is_vector_leaf<ResidualType>{} ||
                    (internal::is_compound_leaf_expression<ResidualType>{} &&
                     internal::is_compound_vector_expression<ResidualType>{}),
                  "Residual type must be a vector expression");

    using FunctorReturnType = decltype(Functor{}(std::declval<const LeafTypes &>()...));
    using check_functor =
      typename internal::factor_functor_check<ResidualType, FunctorReturnType>::type;

 public:
    using const_iterator = typename ArrayType::const_iterator;

    /** Construct with the given measurement and variables. */
    explicit Factor(const MeasType &measurement,
                    std::shared_ptr<FactorVariable<LeafTypes>>... variable_ptrs)
        : measurement{measurement}, variable_ptrs{{std::move(variable_ptrs)...}} {}

    /** Calculate normalized residuals at the given parameters
     *
     * Calls the user-supplied measurement function (the call operator of the functor type
     * `F`) with the given inputs. Then calculates normalized residuals using this
     * factor's measurement, adjusting for uncertainty.
     *
     * @param[in] parameters values for each variable this function operates on
     * @returns Eigen vector of normalized residuals
     */
    template <typename... Params>
    auto evaluate(const Params &... parameters) const noexcept -> ResidualVectorType;

    /** Calculate normalized residuals and Jacobians at the given parameters
     *
     * Calls the user-supplied measurement function (the call operator of the functor type
     * `F`) with the given inputs, which must return an Expression of the inputs.
     * Then calculates normalized residuals and Jacobians using this factor's measurement,
     * adjusting for uncertainty.
     *
     * @param[in] parameters values for each variable this function operates on
     * @returns tuple of (Eigen vector of normalized residuals, Jacobians...)
     */
    template <typename... Params>
    auto evaluateWithJacobians(const Params &... parameters) const noexcept;

    /** Return true if this factor is a zero-noise prior */
    bool isPerfectPrior() const noexcept override {
        return false;
    }

    std::size_t size() const noexcept override {
        return this->variable_ptrs.size();
    }

    const_iterator begin() const noexcept override {
        return this->variable_ptrs.begin();
    }
    const_iterator end() const noexcept override {
        return this->variable_ptrs.end();
    }

 private:
    /** Storage of the measurement */
    MeasType measurement;

    /** Pointers to the variables this factor is linked to */
    ArrayType variable_ptrs;
};

/** Returns a factor with the appropriate template parameters */
template <typename Functor, typename MeasType, typename... LeafTypes>
auto makeFactor(const MeasType &measurement,
                std::shared_ptr<FactorVariable<LeafTypes>>... variable_ptrs) {
    return Factor<Functor, MeasType, LeafTypes...>{measurement,
                                                   std::move(variable_ptrs)...};
}

/** Returns a tuple of expressions.
 *
 * Meant to be used inside functors returning a vector of more than one residual
 * expression.
 */
template <typename... Expressions>
auto residualVector(Expressions &&... exprs) {
    return std::forward_as_tuple(std::forward<Expressions>(exprs)...);
}

}  // namespace wave

#include "impl/Factor_impl.hpp"

#endif  // WAVE_GEOMETRY_FACTOR_HPP
