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
                UnaryExpression<LogMap<ExtraFrame, Rhs>> {
 private:
    using Storage = UnaryExpression<LogMap<ExtraFrame, Rhs>>;

 public:
    // Inherit constructors from UnaryExpression
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
    using Scalar = scalar_t<Rhs>;
    using Jacobian = jacobian_t<RelativeRotation<Val>, Rhs>;
    const auto &phi = val.value();
    // From http://ethaneade.org/exp_diff.pdf
    using std::cos;
    using std::sin;
    const auto theta2 = phi.squaredNorm();
    const auto theta = sqrt(theta2);
    const auto A = sin(theta) / theta;
    const auto B = (Scalar{1} - cos(theta)) / theta2;

    //        if (theta2 > Eigen::NumTraits<Scalar>::epsilon()) {
    return Jacobian::Identity() - Scalar{0.5} * crossMatrix(phi) +
           ((B - Scalar{0.5} * A) / (Scalar{1} - cos(theta))) * crossMatrix(phi) *
             crossMatrix(phi);
    //        } else {
    // @todo small input
    //        }
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_LOGMAP_HPP
