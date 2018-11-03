/**
 * @file
 * @author lkoppel
 * Tests for Euclidean vector objects
 * All types under test must also pass affine_test.
 */

#include "wave/geometry/geometry.hpp"
#include "test.hpp"

template <typename Params>
class VectorTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    using Leaf = typename Params::Leaf;
    using EigenVector = typename wave::internal::traits<Leaf>::ImplType;

    // Used for CHECK_JACOBIANS macro
    static constexpr bool IsFramed = Params::IsFramed;

    using FrameA = typename Params::FrameA;
    using FrameB = typename Params::FrameB;
    using FrameC = typename Params::FrameC;

    // Convenience types
    template <typename T, typename... F>
    using Framed = typename Params::template Framed<T, F...>;

    using LeafAAB = Framed<Leaf, FrameA, FrameA, FrameB>;
    using LeafABC = Framed<Leaf, FrameA, FrameB, FrameC>;
    using LeafABA = Framed<Leaf, FrameA, FrameB, FrameA>;
    using LeafACA = Framed<Leaf, FrameA, FrameC, FrameA>;
    using LeafACB = Framed<Leaf, FrameA, FrameC, FrameB>;
    using LeafAAC = Framed<Leaf, FrameA, FrameA, FrameC>;
    using LeafBAC = Framed<Leaf, FrameB, FrameA, FrameC>;

    using ZeroLeafAAB = wave::Zero<LeafAAB>;
    using ZeroLeafABC = wave::Zero<LeafABC>;

    // Static traits checks
    TICK_TRAIT_CHECK(wave::internal::is_vector_leaf<Leaf>);
    TICK_TRAIT_CHECK(wave::internal::is_vector_leaf<LeafAAB>);
    TICK_TRAIT_CHECK(wave::internal::is_leaf_expression<ZeroLeafAAB>);
    TICK_TRAIT_CHECK(wave::internal::is_unary_expression<
                     wave::FrameCast<FrameC, FrameB, FrameA, LeafAAB>>);
};

// Use a type-parametrized test case, meaning we can instantiate it with types later
// without knowing the types in advance.
// See https://github.com/google/googletest/blob/master/googletest/docs/advanced.md
TYPED_TEST_CASE_P(VectorTest);

TYPED_TEST_P(VectorTest, addWithFrameCast) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto expr = wave::frame_cast<typename TestFixture::FrameA,
                                       typename TestFixture::FrameC,
                                       typename TestFixture::FrameA>(
      t1 + wave::frame_cast<typename TestFixture::FrameA,
                            typename TestFixture::FrameB,
                            typename TestFixture::FrameC>(t1));

    const auto result = typename TestFixture::LeafACA{expr};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() * 2};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(false, expr, t1);
}

TYPED_TEST_P(VectorTest, negate) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto result = typename TestFixture::LeafABA{-t1};
    const auto eigen_result = typename TestFixture::EigenVector{-(t1.value())};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(true, -t1, t1);
}

TYPED_TEST_P(VectorTest, subtract) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafACB::Random();
    const auto result = typename TestFixture::LeafAAC{t1 - t2};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() - t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 - t2, t1, t2);
}

TYPED_TEST_P(VectorTest, subtractFlippedFrames) {
    const auto t1 = TestFixture::LeafABC::Random();
    const auto t2 = TestFixture::LeafABA::Random();
    const auto result = typename TestFixture::LeafAAC{t1 - t2};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() - t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 - t2, t1, t2);
}

TYPED_TEST_P(VectorTest, squaredNormExpr) {
    const auto t1 = TestFixture::LeafAAB::Random();

    // Note .squaredNorm() returns an expression, which here is implicitly converted to a
    // double
    const double result = t1.squaredNorm();
    const auto eigen_result = t1.value().squaredNorm();

    ASSERT_DOUBLE_EQ(eigen_result, result);

    // We could also do this
    const auto result2 = t1.squaredNorm().eval();
    ASSERT_DOUBLE_EQ(eigen_result, result2);

    CHECK_JACOBIANS(true, t1.squaredNorm(), t1);
}

TYPED_TEST_P(VectorTest, normExpr) {
    const auto t1 = TestFixture::LeafAAB::Random();

    // Note .norm() returns an expression, which here is implicitly converted to a double
    const double result = t1.norm();
    const auto eigen_result = t1.value().norm();

    ASSERT_DOUBLE_EQ(eigen_result, result);
    CHECK_JACOBIANS(true, t1.norm(), t1);
}

TYPED_TEST_P(VectorTest, normalizeExpr) {
    const auto t1 = TestFixture::LeafAAB::Random();

    // Note .norm() returns an expression, which here is implicitly converted to a double
    const auto expr = t1.normalized();
    const auto eigen_result = t1.value().normalized();

    EXPECT_APPROX(eigen_result, expr.eval().value());
    CHECK_JACOBIANS(true, expr, t1);
}

TYPED_TEST_P(VectorTest, dotProduct) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();

    // Note .norm() returns an expression, which here is implicitly converted to a double
    const auto expr = dot(t1, t2);
    const double eigen_result = t1.value().dot(t2.value());

    EXPECT_DOUBLE_EQ(eigen_result, expr.eval().value());
    CHECK_JACOBIANS(TestFixture::IsFramed, expr, t1, t2);
}

TYPED_TEST_P(VectorTest, addNorms) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();

    const auto result = eval(t1.norm() + t2.norm());
    const auto eigen_result = t1.value().norm() + t2.value().norm();

    ASSERT_DOUBLE_EQ(eigen_result, result);
    CHECK_JACOBIANS(TestFixture::IsFramed, t1.norm() + t2.norm(), t1, t2);
}

TYPED_TEST_P(VectorTest, scaleLeft) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto a = wave::uniformRandom<typename TestFixture::Scalar>(-3.0, 3.0);
    const auto result = typename TestFixture::LeafAAB{a * t1};
    const auto eigen_result = typename TestFixture::EigenVector{a * t1.value()};
    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(true, a * t1, a, t1);
}

TYPED_TEST_P(VectorTest, scaleRight) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto a = wave::uniformRandom<typename TestFixture::Scalar>(-3.0, 3.0);
    const auto result = typename TestFixture::LeafAAB{t1 * a};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() * a};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(true, t1 * a, t1, a);
}

TYPED_TEST_P(VectorTest, scaleDivideRight) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto a = wave::uniformRandom<typename TestFixture::Scalar>(-3.0, 3.0);
    const auto result = typename TestFixture::LeafAAB{t1 / a};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() / a};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(true, t1 * a, t1, a);
}

TYPED_TEST_P(VectorTest, scaleByExpressionLeft) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = typename TestFixture::LeafAAB{t2.norm() * t1};
    const auto eigen_result =
      typename TestFixture::EigenVector{t2.value().norm() * t1.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t2.norm() * t1, t2, t1);
}

TYPED_TEST_P(VectorTest, scaleByExpressionRight) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = typename TestFixture::LeafAAB{t1 * t2.norm()};
    const auto eigen_result =
      typename TestFixture::EigenVector{t1.value() * t2.value().norm()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 * t2.norm(), t1, t2);
}

TYPED_TEST_P(VectorTest, scaleDivideByExpressionRight) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = typename TestFixture::LeafAAB{t1 / t2.norm()};
    const auto eigen_result =
      typename TestFixture::EigenVector{t1.value() / t2.value().norm()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 / t2.norm(), t1, t2);
}

// When adding a test it must also be added to the REGISTER_TYPED_TEST_CASE_P call below.
// Yes, it's redundant; apparently the drawback of using type-parameterized tests.
REGISTER_TYPED_TEST_CASE_P(VectorTest,
                           addWithFrameCast,
                           negate,
                           subtract,
                           subtractFlippedFrames,
                           squaredNormExpr,
                           normExpr,
                           normalizeExpr,
                           dotProduct,
                           addNorms,
                           scaleLeft,
                           scaleRight,
                           scaleDivideRight,
                           scaleByExpressionLeft,
                           scaleByExpressionRight,
                           scaleDivideByExpressionRight);
