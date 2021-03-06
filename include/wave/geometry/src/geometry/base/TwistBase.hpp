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
    // The blocks are stored in this order:
    enum : int { Rotation, Translation };

    template <typename T>
    using BaseTmpl = TwistBase<T>;
};

/** Takes exponential map of an se(3) element */
template <typename R>
auto exp(const TwistBase<R> &rhs) {
    return ExpMap<internal::cr_arg_t<R>>{rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUE(exp, ExpMap, TwistBase)

}  // namespace wave
#endif  // WAVE_GEOMETRY_TWISTBASE_HPP
