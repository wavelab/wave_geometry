/**
 * @file
 */

#ifndef WAVE_GEOMETRY_EXPMAP_HPP
#define WAVE_GEOMETRY_EXPMAP_HPP

namespace wave {

/** Expression representing the exponential map of a relative rotation
 *
 * @tparam Rhs The relative rotation expression in so(3)
 */
template <typename Rhs>
struct ExpMap
    : internal::base_tmpl_t<typename internal::eval_traits<Rhs>::ExpType, ExpMap<Rhs>>,
      UnaryExpression<ExpMap<Rhs>> {
 private:
    using Storage = UnaryExpression<ExpMap<Rhs>>;

 public:
    // Inherit constructors from UnaryExpression
    using Storage::Storage;
};

namespace internal {

template <typename Rhs>
struct traits<ExpMap<Rhs>> : unary_traits_base<ExpMap<Rhs>> {
    using OutputFunctor = WrapWithFrames<LeftFrameOf<Rhs>, LeftFrameOf<Rhs>>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_EXPMAP_HPP
