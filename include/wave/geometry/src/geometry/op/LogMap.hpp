/**
 * @file
 */

#ifndef WAVE_GEOMETRY_LOGMAP_HPP
#define WAVE_GEOMETRY_LOGMAP_HPP

namespace wave {

/** Expression representing the logarithmic map of a rotation
 *
 * @tparam Rhs The rotation expression in SO(3)
 */
template <typename ExtraFrame, typename Rhs>
struct LogMap : internal::base_tmpl_t<typename internal::eval_traits<Rhs>::TangentType,
                                      LogMap<ExtraFrame, Rhs>>,
                internal::unary_storage_for<LogMap<ExtraFrame, Rhs>> {
 private:
    using Storage = internal::unary_storage_for<LogMap<ExtraFrame, Rhs>>;

 public:
    // Inherit constructors from UnaryStorage
    using Storage::Storage;
};

namespace internal {

template <typename ExtraFrame, typename Rhs>
struct traits<LogMap<ExtraFrame, Rhs>>
    : unary_traits_base_tag<LogMap<ExtraFrame, Rhs>, expr<LogMap>> {
    using OutputFunctor = WrapWithFrames<LeftFrameOf<Rhs>, LeftFrameOf<Rhs>, ExtraFrame>;
};

/** Jacobian of logmap of any rotation
 *
 * It only uses the result, thus is independent of the rotation parametrization.
 * */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<LogMap>,
                  const RelativeRotation<Val> &val,
                  const RotationBase<Rhs> &) -> jacobian_t<RelativeRotation<Val>, Rhs> {
    return ::wave::jacobianOfRotationLogMap(val.value());
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_LOGMAP_HPP
