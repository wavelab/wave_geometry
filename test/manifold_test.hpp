/**
 * @file
 * @author lkoppel
 * Fixture and tests of manifold expressions
 *
 * Tests correctness and consistency of manifold operations, both with and without frames.
 *
 * This test covers operations common to both SO(3) and SE(3), such as logmap and expmap.
 *
 * Some tests in this fixture refer to equations in Bloesch, Michael, et al.
 * "A Primer on the Differential Calculus of 3D Orientations." arXiv:1606.05285 (2016).
 *
 * The tests are included in this header file so that instantiations for different types
 * (e.g. RotationMd and RigidTransformMd) can be split into different translation units.
 */

#ifndef WAVE_GEOMETRY_MANIFOLD_TEST_HPP
#define WAVE_GEOMETRY_MANIFOLD_TEST_HPP

#include "wave/geometry/geometry.hpp"
#include "test.hpp"

template <typename Params>
class ManifoldTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    using Leaf = typename Params::Leaf;
    using ImplType = typename wave::internal::traits<Leaf>::ImplType;

    // Used for CHECK_JACOBIANS macro
    static constexpr bool IsFramed = Params::IsFramed;

    using FrameA = typename Params::FrameA;
    using FrameB = typename Params::FrameB;
    using FrameC = typename Params::FrameC;

    // Define different rotation types with the same scalar, to test composition
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Quaternion = Eigen::Quaternion<Scalar>;
    using AngleAxis = Eigen::AngleAxis<Scalar>;
    using RotationA = wave::AngleAxisRotation<AngleAxis>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using RelLeaf = typename wave::internal::traits<Leaf>::TangentType;
    using Translation = wave::Translation<Vector3>;

    // Convenience framed types
    template <typename T, typename... F>
    using Framed = typename Params::template Framed<T, F...>;

    using LeafAA = Framed<Leaf, FrameA, FrameA>;
    using LeafAB = Framed<Leaf, FrameA, FrameB>;
    using LeafBA = Framed<Leaf, FrameB, FrameA>;
    using LeafBC = Framed<Leaf, FrameB, FrameC>;
    using LeafAC = Framed<Leaf, FrameA, FrameC>;
    using RelLeafAAB = Framed<RelLeaf, FrameA, FrameA, FrameB>;

    const Scalar dummy_prec = 10 * Eigen::NumTraits<Scalar>::dummy_precision();

    // Static traits checks
    TICK_TRAIT_CHECK(wave::internal::is_leaf_expression<LeafAB>);
    TICK_TRAIT_CHECK(wave::internal::is_derived_transform<LeafAB>);
    TICK_TRAIT_CHECK(wave::internal::is_binary_expression<wave::Compose<Leaf, Leaf>>);
    TICK_TRAIT_CHECK(
      wave::internal::is_binary_expression<wave::BoxPlus<LeafAB, RelLeafAAB>>);
    TICK_TRAIT_CHECK(wave::internal::is_unary_expression<wave::BoxMinus<LeafAB, LeafAB>>);
};

// Use a type-parametrized test case, meaning we can instantiate it with types later
// without knowing the types in advance.
// See https://github.com/google/googletest/blob/master/googletest/docs/advanced.md
TYPED_TEST_CASE_P(ManifoldTest);

// When adding a test it must also be added to the REGISTER_TYPED_TEST_CASE_P call below.
// Yes, it's redundant; apparently the drawback of using type-parameterized tests.

// Check that the result of Random() is the same type
TYPED_TEST_P(ManifoldTest, constructRandom) {
    auto r = TestFixture::LeafAB::Random();
    static_assert(std::is_same<typename TestFixture::LeafAB, decltype(r)>{},
                  "Random() returns non-plain type");
}

TYPED_TEST_P(ManifoldTest, isApproxTrivial) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = typename TestFixture::LeafBA{r1.value()};

    // Note isApprox does not care about frames
    EXPECT_TRUE(r1.isApprox(r2));
}

TYPED_TEST_P(ManifoldTest, isApprox) {
    const auto r1 = TestFixture::LeafAB::Random();
    auto rel = TestFixture::RelLeafAAB::Random();
    rel.value() *= 0.01;  // @todo scaling expression
    const auto r2 = eval(r1 + rel);

    EXPECT_FALSE(r1.isApprox(r2));
    EXPECT_TRUE(r1.isApprox(r2, 0.01));
}

TYPED_TEST_P(ManifoldTest, inverseIdentity) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = typename TestFixture::LeafBA{inverse(r1)};

    const auto res = typename TestFixture::LeafAA{r1 * r2};

    EXPECT_APPROX(TestFixture::LeafAA::Identity(), res);
}

TYPED_TEST_P(ManifoldTest, composeWithIdentity) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = TestFixture::LeafBA::Identity();
    const auto res = typename TestFixture::LeafAA{r1 * r2};
    EXPECT_APPROX(r1, res);
    CHECK_JACOBIANS(true, r1 * r2, r1, r2);
}

TYPED_TEST_P(ManifoldTest, composeWithIdentityLeft) {
    const auto r1 = TestFixture::LeafAB::Identity();
    const auto r2 = TestFixture::LeafBA::Random();
    const auto res = typename TestFixture::LeafAA{r1 * r2};
    EXPECT_APPROX(r2, res);
    CHECK_JACOBIANS(true, r1 * r2, r1, r2);
}

TYPED_TEST_P(ManifoldTest, boxPlusZero) {
    // Bloesch Equation 16
    const auto r1 = TestFixture::LeafAB::Random();
    const auto rel = wave::Zero<typename TestFixture::RelLeafAAB>{};

    const auto result = eval(r1 + rel);
    EXPECT_APPROX(r1, result);
    CHECK_JACOBIANS(true, r1 + rel, r1, rel);
}

TYPED_TEST_P(ManifoldTest, boxPlusMinus) {
    // Bloesch Equation 17
    const auto r1 = TestFixture::LeafAB::Random();
    auto rel = TestFixture::RelLeafAAB::Random();

    const auto result = eval((r1 + rel) - r1);
    EXPECT_APPROX_PREC(rel, result, this->dummy_prec * 10);
}

TYPED_TEST_P(ManifoldTest, boxMinusPlus) {
    // Bloesch Equation 18
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = TestFixture::LeafAB::Random();

    const auto result = typename TestFixture::LeafAB{r1 + (r2 - r1)};
    EXPECT_APPROX_PREC(r2, result, this->dummy_prec * 10);
}

TYPED_TEST_P(ManifoldTest, consistentExpLog) {
    // Special case: for AngleAxis this operation requires converting from angle-axis and
    // back, so the precision is a bit worse
    auto prec = this->dummy_prec;
    if (std::is_same<typename TestFixture::Leaf, typename TestFixture::RotationA>{}) {
        prec *= 10;
    }

    // Bloesch section D
    const auto r1 = TestFixture::LeafAA::Random();

    const auto result = typename TestFixture::LeafAA{exp(eval(log(r1)))};
    EXPECT_APPROX_PREC(r1, result, prec);
}

TYPED_TEST_P(ManifoldTest, logMapJacobian) {
    // Use a small rotation for more accurate numerical jacobians
    auto rel = TestFixture::RelLeafAAB::Random();
    rel.value() *= 0.02;
    // @todo use scaling expression
    const auto r1 = typename TestFixture::LeafAB{
      wave::frame_cast<typename TestFixture::FrameA, typename TestFixture::FrameB>(
        exp(rel))};
    const auto res = typename TestFixture::RelLeafAAB{log(r1)};
    (void) res;  // just checking evaluability

    CHECK_JACOBIANS(true, log(r1), r1);
}


TYPED_TEST_P(ManifoldTest, expMapJacobian) {
    auto rel = TestFixture::RelLeafAAB::Random();
    rel.value() *= 0.02;
    CHECK_JACOBIANS(true, exp(rel), rel);
}


TYPED_TEST_P(ManifoldTest, expMapJacobianNearZero) {
    using S = typename TestFixture::Scalar;
    auto rel = TestFixture::RelLeafAAB::Zero().eval();
    rel.value()[2] = S{1e-14};
    CHECK_JACOBIANS(true, exp(rel), rel);
}

TYPED_TEST_P(ManifoldTest, boxPlusJacobian) {
    const auto r1 = TestFixture::LeafAB::Random();
    auto rel = TestFixture::RelLeafAAB::Random();
    rel.value() *= 0.02;
    CHECK_JACOBIANS(true, r1 + rel, r1, rel);
}

TYPED_TEST_P(ManifoldTest, boxMinusJacobian) {
    const auto r1 = TestFixture::LeafAB::Random();
    auto rel = TestFixture::RelLeafAAB::Random();
    rel.value() *= 0.02;
    const auto r2 = typename TestFixture::LeafAB{r1 + rel};
    CHECK_JACOBIANS(false, r1 - r2, r1, r2);
}

// Register the test case (having to list all the tests again)
REGISTER_TYPED_TEST_CASE_P(ManifoldTest,
                           constructRandom,
                           isApproxTrivial,
                           isApprox,
                           inverseIdentity,
                           composeWithIdentity,
                           composeWithIdentityLeft,
                           boxPlusZero,
                           boxPlusMinus,
                           boxMinusPlus,
                           consistentExpLog,
                           logMapJacobian,
                           expMapJacobian,
                           expMapJacobianNearZero,
                           boxPlusJacobian,
                           boxMinusJacobian);

#endif  // WAVE_GEOMETRY_MANIFOLD_TEST_HPP
