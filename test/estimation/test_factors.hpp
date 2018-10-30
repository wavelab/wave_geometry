/**
 * @file
 * @author lkoppel
 * Examples of Factor and FactorVariable instances
 */

#ifndef WAVE_GEOMETRY_TEST_FACTORS_HPP
#define WAVE_GEOMETRY_TEST_FACTORS_HPP

#include "wave/geometry/estimation.hpp"
#include "../test.hpp"

namespace example {

/** Sample functor for calculating distance
 *
 * Notice how each parameter corresponds to strongly-typed variables and
 * matrices, instead of something like `double **`.
 */
struct DistanceFunctor {
    template <typename T, typename U>
    auto operator()(const wave::TranslationBase<T> &a,
                    const wave::TranslationBase<U> &b) const {
        static_assert(wave::internal::same_frames<T, U>{}, "Frames must be the same");
        return (a.derived() - b.derived()).norm();
    }
};

template <typename RangeExpr = wave::Scalar<double>,
          typename BearingExpr = wave::Scalar<double>>
struct RangeBearing
    : wave::CompoundVectorBase<RangeBearing<RangeExpr, BearingExpr>>,
      wave::NaryStorage<RangeBearing<RangeExpr, BearingExpr>, RangeExpr, BearingExpr> {
    using Storage =
      wave::NaryStorage<RangeBearing<RangeExpr, BearingExpr>, RangeExpr, BearingExpr>;
    using Storage::Storage;
    using Storage::operator=;

    auto range() {
        return this->template get<0>();
    }

    const auto &range() const {
        return this->template get<0>();
    }

    auto bearing() {
        return this->template get<1>();
    }

    const auto &bearing() const {
        return this->template get<1>();
    }
};

using RangeBearingd = RangeBearing<>;

/** Sample functor calculating two values: distance and angle difference */
struct RangeBearingFunctor {
    template <typename A, typename B>
    auto operator()(const wave::RigidTransformBase<A> &a,
                    const wave::TranslationBase<B> &b) const {
        static_assert(std::is_same<wave::LeftFrameOf<A>, wave::LeftFrameOf<B>>{},
                      "Left frames must be the same");
        static_assert(std::is_same<wave::LeftFrameOf<A>, wave::MiddleFrameOf<B>>{},
                      "Left frames must be the same");
        const auto &T_WB = a.derived();
        const auto &W_v_WL = b.derived();
        auto &&B_v_Bz = wave::TranslationFd<wave::RightFrameOf<A>,
                                            wave::RightFrameOf<A>,
                                            wave::RightFrameOf<B>>{0., 0., 1.};
        auto &&range = (T_WB.translation() - W_v_WL).norm();
        auto &&bearing =
          acos(dot(std::move(B_v_Bz), (inverse(T_WB) * W_v_WL.derived()).normalized()));
        return wave::internal::makeCompound<RangeBearing>(std::move(range),
                                                          std::move(bearing));
    }
};

}  // namespace example


namespace wave {
namespace internal {
template <typename T1, typename T2>
struct traits<example::RangeBearing<T1, T2>>
    : compound_vector_traits_base<example::RangeBearing<T1, T2>, T1, T2>,
      frameable_vector_traits {};

}  // namespace internal
}  // namespace wave


#endif  // WAVE_GEOMETRY_TEST_FACTORS_HPP
