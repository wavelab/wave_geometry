/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_FACTOR_IMPL_HPP
#define WAVE_GEOMETRY_FACTOR_IMPL_HPP

namespace wave {
namespace internal {

template <typename NewHead, typename Head, typename... Tail, int... Is>
auto replaceFirstTupleElementImpl(NewHead &&new_head,
                                  std::tuple<Head, Tail...> &&tuple,
                                  tmp::index_sequence<Is...>) {
    return std::make_tuple(new_head, std::get<Is>(std::move(tuple))...);
}

/** Replaces the first element in a tuple with something else */
template <typename NewHead, typename Head, typename... Tail>
auto replaceFirstTupleElement(NewHead &&new_head, std::tuple<Head, Tail...> &&tuple) {
    using IndicesExcludingFirst = tmp::make_index_sequence<sizeof...(Tail), 1>;
    return replaceFirstTupleElementImpl(
      std::move(new_head), std::move(tuple), IndicesExcludingFirst{});
}


}  // namespace internal

template <typename Functor, typename MeasType, typename... LeafTypes>
template <typename... Params>
auto Factor<Functor, MeasType, LeafTypes...>::evaluate(const Params &... parameters) const
  noexcept -> ResidualVectorType {
    // Get expression for the measurement function
    const auto expr = Functor{}(parameters...);
    const auto residual_vec = ResidualVectorType{
      valueAsVector(internal::adl{}, eval(expr - this->measurement.value))};

    // Calculate the normalized residual
    const auto &L = this->measurement.noise.inverseSqrtCov();
    return ResidualVectorType{L * residual_vec};
}

template <typename Functor, typename MeasType, typename... LeafTypes>
template <typename... Params>
auto Factor<Functor, MeasType, LeafTypes...>::evaluateWithJacobians(
  const Params &... parameters) const noexcept {
    // Get expression and Jacobians of the measurement function
    const auto expr = Functor{}(parameters...);

    auto value_and_jac_tuple = expr.evalWithJacobians(parameters...);

    const auto residual_vec = ResidualVectorType{valueAsVector(
      internal::adl{}, eval(std::get<0>(value_and_jac_tuple) - this->measurement.value))};

    // Calculate the normalized residual and place it into the result
    const auto &L = this->measurement.noise.inverseSqrtCov();
    return internal::replaceFirstTupleElement(ResidualVectorType{L * residual_vec},
                                              std::move(value_and_jac_tuple));
}

}  // namespace wave

#endif  // WAVE_GEOMETRY_FACTOR_IMPL_HPP
