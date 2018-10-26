#include "test.hpp"
#include "wave/geometry/geometry.hpp"

template <typename Params>
class CompoundTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    using Leaf = typename Params::Leaf;
    using Translation = wave::Translation<Eigen::Matrix<Scalar, 3, 1>>;


    // Used for CHECK_JACOBIANS macro
    static constexpr bool IsFramed = Params::IsFramed;

    using FrameA = typename Params::FrameA;
    using FrameB = typename Params::FrameB;
    using FrameC = typename Params::FrameC;

    // Convenience framed types
    template <typename T, typename... F>
    using Framed = typename Params::template Framed<T, F...>;

    using RotAA = Framed<Leaf, FrameA, FrameA>;
    using RotAB = Framed<Leaf, FrameA, FrameB>;
    using RotBC = Framed<Leaf, FrameB, FrameC>;
    using RotAC = Framed<Leaf, FrameA, FrameC>;
    using RotCA = Framed<Leaf, FrameC, FrameA>;
    using TransAAB = Framed<Translation, FrameA, FrameA, FrameB>;
    using TransBAB = Framed<Translation, FrameB, FrameA, FrameB>;
    using TransCAB = Framed<Translation, FrameC, FrameA, FrameB>;


    // Static traits checks
};
