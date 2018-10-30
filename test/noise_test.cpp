#include "test.hpp"
#include "wave/geometry/estimation.hpp"

template <typename Params>
class NoiseTest : public testing::Test {
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

    using LeafAA = Framed<Leaf, FrameA, FrameA>;
    using LeafAB = Framed<Leaf, FrameA, FrameB>;
    using LeafBC = Framed<Leaf, FrameB, FrameC>;
    using LeafAC = Framed<Leaf, FrameA, FrameC>;
    using LeafCA = Framed<Leaf, FrameC, FrameA>;
    using TransAAB = Framed<Translation, FrameA, FrameA, FrameB>;
    using TransBAB = Framed<Translation, FrameB, FrameA, FrameB>;
    using TransCAB = Framed<Translation, FrameC, FrameA, FrameB>;

    using TangentAA = typename wave::internal::traits<LeafAA>::TangentType;
    using Identity = wave::internal::identity_t<LeafAA>;
    using Block = wave::BlockMatrix<LeafAA, LeafAA>;


    // Static traits checks
};

// The list of implementation types to run each test case on
using LeafTypes = test_types_list<wave::RotationQd>;

// The following tests will be built for each type in LeafTypes
TYPED_TEST_CASE(NoiseTest, LeafTypes);


TYPED_TEST(NoiseTest, diagonalNoise) {
    using Block = typename TestFixture::Block;

    const auto stddev = TestFixture::TangentAA::Random().value();
    const auto expected_cov =
      Block{Block{stddev.asDiagonal()} * Block{stddev.asDiagonal()}};

    const auto n = wave::DiagonalNoise<typename TestFixture::LeafAA>::FromStdDev(stddev);

    EXPECT_APPROX(expected_cov, n.covariance().toDenseMatrix());
    EXPECT_APPROX(
      expected_cov.inverse(),
      n.inverseSqrtCov().toDenseMatrix() * n.inverseSqrtCov().toDenseMatrix());
}

TYPED_TEST(NoiseTest, singleNoise) {
    const auto stddev = 1.1;

    const auto noise =
      wave::DiagonalNoise<wave::Scalar<typename TestFixture::Scalar>>::FromStdDev(stddev);
    EXPECT_DOUBLE_EQ(stddev * stddev, noise.covariance().toDenseMatrix()[0]);
    EXPECT_DOUBLE_EQ(1. / stddev, noise.inverseSqrtCov().toDenseMatrix()[0]);
}

TYPED_TEST(NoiseTest, fullNoise) {
    // Construct a random positive-definite matrix
    const auto r = TestFixture::Block::Random().eval();
    const auto n = TestFixture::Block::RowsAtCompileTime;
    const auto cov =
      ((r + r.transpose()) / 2 + n * TestFixture::Block::Identity()).eval();
    const auto A = (r * r).inverse().eval();

    const auto noise = wave::FullNoise<typename TestFixture::LeafAA>::FromCovariance(cov);

    EXPECT_APPROX(cov, noise.covariance());
    EXPECT_APPROX(TestFixture::Block::Identity(),
                  noise.inverseSqrtCov() * cov * noise.inverseSqrtCov());
    EXPECT_APPROX(TestFixture::Block::Identity(), r * A * r);
}
