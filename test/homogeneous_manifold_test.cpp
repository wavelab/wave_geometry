/**
 * @file
 * @author lkoppel
 * Tests identities of manifold expressions on homogeneous points
 *
 * @see manifold_test.cpp
 */
#include "manifold_test.cpp"

namespace wave {
// Workaround for applying ManifoldTest to homogeneous points:
// Define functions which let us test the implementation, even though these "inverse" and
// "exp", etc.,  are not exposed to the user.

template <typename R>
auto inverse(const HomogeneousPointBase<R> &rhs) {
    return Inverse<internal::cr_arg_t<R>>{rhs.derived()};
}

template <typename L, typename R>
auto operator*(const HomogeneousPointBase<L> &lhs, const HomogeneousPointBase<R> &rhs) {
    return Compose<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                 rhs.derived()};
}

template <typename R>
auto exp(const TranslationBase<R> &rhs) {
    return ExpMap<internal::cr_arg_t<R>>{rhs.derived()};
}

template <typename R>
auto log(const HomogeneousPointBase<R> &rhs) {
    return LogMap<RightFrameOf<R>, internal::cr_arg_t<R>>{rhs.derived()};
}

}  // namespace wave
