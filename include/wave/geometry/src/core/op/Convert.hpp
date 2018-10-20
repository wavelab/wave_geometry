/**
 * @file
 */

#ifndef WAVE_GEOMETRY_CONVERT_HPP
#define WAVE_GEOMETRY_CONVERT_HPP

namespace wave {
/**  Expression representing a conversion from one representation to another
 *
 * @tparam ToDerived The desired leaf type to convert to
 * @tparam FromDerived The expression type to convert from
 */
template <typename ToDerived, typename FromDerived>
struct Convert : internal::base_tmpl_t<FromDerived, Convert<ToDerived, FromDerived>>,
                 UnaryExpression<Convert<ToDerived, FromDerived>> {
    using Storage = UnaryExpression<Convert<ToDerived, FromDerived>>;
    using Storage::Storage;
};

namespace internal {

template <typename ToDerived, typename FromDerived>
struct traits<Convert<ToDerived, FromDerived>>
    : unary_traits_base<Convert<ToDerived, FromDerived>> {
    using OutputFunctor = typename traits<FromDerived>::OutputFunctor;
};

/** Jacobian implementation for all Convert */
template <typename To, typename Res, typename From>
auto jacobianImpl(expr<Convert, To>, const Res &, const From &) {
    return identity_t<Res>{};
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_CONVERT_HPP
