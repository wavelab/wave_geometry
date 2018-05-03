/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_TWISTBASE_HPP
#define WAVE_GEOMETRY_TWISTBASE_HPP

namespace wave {

/** Base class for expressions representing a difference in rigid transforms, in se(3).
 *
 * This class is inherited by expressions types which represent an element of @f$ se(3)
 * @f$, and which evaluate to Twist.
 *
 * @see Twist
 * */
template <typename Derived>
struct TwistBase : public VectorBase<Derived> {
    template <typename T>
    using BaseTmpl = TwistBase<T>;
};

/** Takes exponential map of an se(3) element */
template <typename Rhs>
auto exp(const TwistBase<Rhs> &rhs) -> ExpMap<Rhs> {
    return ExpMap<Rhs>{rhs.derived()};
}
// Overload for rvalue
template <typename Rhs>
auto inverse(TwistBase<Rhs> &&rhs) -> ExpMap<internal::arg_t<Rhs>> {
    return ExpMap<internal::arg_t<Rhs>>{std::move(rhs).derived()};
}
}  // namespace wave
#endif  // WAVE_GEOMETRY_TWISTBASE_HPP
