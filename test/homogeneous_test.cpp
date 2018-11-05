/**
 * @file
 * @author lkoppel
 * Tests for homogeneous points
 */

#include "wave/geometry/geometry.hpp"
#include "test.hpp"

template <typename Params>
class HomogeneousTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    // The affine leaf (might be a point or vector)
    using Leaf = typename Params::Leaf;
    // The affine (actually Euclidean) vector that is the difference type of Leaf
    using Vector = typename wave::internal::traits<Leaf>::TangentType;

    using EigenVector3 = Eigen::Matrix<Scalar, 3, 1>;
    using EigenVector4 = Eigen::Matrix<Scalar, 4, 1>;


    // Used for CHECK_JACOBIANS macro
    static constexpr bool IsFramed = Params::IsFramed;
    static constexpr bool DifferentLeafAndVector = !std::is_same<Leaf, Vector>{};
    static constexpr bool IsFramedOrDifferent = IsFramed || DifferentLeafAndVector;

    using FrameA = typename Params::FrameA;
    using FrameB = typename Params::FrameB;
    using FrameC = typename Params::FrameC;

    // Convenience types
    template <typename T, typename... F>
    using Framed = typename Params::template Framed<T, F...>;

    // Choose either A_P_B for points or A_v_AB for vectors
    template <typename T, typename FL, typename FR>
    using FramedPt = std::conditional_t<wave::internal::has_three_decorators<T>{},
                                        Framed<T, FL, FL, FR>,
                                        Framed<T, FL, FR>>;

    using LeafAA = FramedPt<Leaf, FrameA, FrameA>;
    using LeafAB = FramedPt<Leaf, FrameA, FrameB>;
    using LeafAC = FramedPt<Leaf, FrameA, FrameC>;
    using LeafBC = FramedPt<Leaf, FrameB, FrameC>;
    using VectorAAB = Framed<Vector, FrameA, FrameA, FrameB>;
    using VectorABC = Framed<Vector, FrameA, FrameB, FrameC>;
    using VectorABA = Framed<Vector, FrameA, FrameB, FrameA>;
    using VectorACA = Framed<Vector, FrameA, FrameC, FrameA>;
    using VectorAAC = Framed<Vector, FrameA, FrameA, FrameC>;
    using VectorACB = Framed<Vector, FrameA, FrameC, FrameB>;
    using VectorBAC = Framed<Vector, FrameB, FrameA, FrameC>;

    using ZeroLeafAB = wave::Zero<LeafAB>;
    using ZeroLeafAC = wave::Zero<LeafAC>;
    using ZeroVectorAAB = wave::Zero<VectorAAB>;
    using ZeroVectorABC = wave::Zero<VectorABC>;

    // Static traits checks
    // Currently internal vector_leaf trait applies to affine points as well
    TICK_TRAIT_CHECK(wave::internal::is_vector_leaf<Leaf>);
    TICK_TRAIT_CHECK(wave::internal::is_vector_leaf<LeafAB>);
    TICK_TRAIT_CHECK(wave::internal::is_leaf_expression<ZeroLeafAB>);
    TICK_TRAIT_CHECK(
      wave::internal::is_unary_expression<wave::FrameCast<FrameC, FrameA, LeafAB>>);
};

// Use a type-parametrized test case, meaning we can instantiate it with types later
// without knowing the types in advance.
// See https://github.com/google/googletest/blob/master/googletest/docs/advanced.md
TYPED_TEST_CASE_P(HomogeneousTest);

// Direct construct from an Eigen vector
TYPED_TEST_P(HomogeneousTest, constructFromVector) {
    const auto eigen_vec =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Random()};
    const auto t = typename TestFixture::LeafAB{eigen_vec};
    EXPECT_APPROX(eigen_vec, t.value());
}

// Direct construct from a temporary Eigen vector
TYPED_TEST_P(HomogeneousTest, constructFromTempVector) {
    const auto eigen_vec =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Zero()};
    const auto t = typename TestFixture::LeafAB{
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Zero()}};
    EXPECT_APPROX(eigen_vec, t.value());
}

// Direct construct from a non-plain Eigen expression
TYPED_TEST_P(HomogeneousTest, constructFromEigenExpression) {
    const auto eigen_vec =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Random()};
    const auto t = typename TestFixture::LeafAB{-eigen_vec};
    EXPECT_APPROX((-eigen_vec).eval(), t.value());
}

// Direct construct from three scalars (only for 3-vector case)
TYPED_TEST_P(HomogeneousTest, constructFromCoeffs) {
    const auto eigen_vec = TestFixture::EigenVector4::Random().normalized().eval();

    const auto t = typename TestFixture::LeafAB{
      eigen_vec[0], eigen_vec[1], eigen_vec[2], eigen_vec[3]};
    EXPECT_APPROX(eigen_vec, t.value());
}

// Construct from an object of the same type
TYPED_TEST_P(HomogeneousTest, copyConstructBraces) {
    const auto eigen_vec =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Random()};
    const typename TestFixture::LeafAB t1{eigen_vec};

    const typename TestFixture::LeafAB t2{t1};
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Construct from an object of the same type
TYPED_TEST_P(HomogeneousTest, copyConstructEquals) {
    const auto eigen_vec =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Random()};
    const typename TestFixture::LeafAB t1{eigen_vec};

    const typename TestFixture::LeafAB t2 = t1;
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Assign from an object of the same type
TYPED_TEST_P(HomogeneousTest, copyAssign) {
    const auto eigen_vec =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Random()};
    typename TestFixture::LeafAB t1{eigen_vec}, t2{};

    t2 = t1;
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Assign from a temporary object of the same type
TYPED_TEST_P(HomogeneousTest, moveAssign) {
    const auto eigen_vec =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Random()};
    typename TestFixture::LeafAB t2;

    t2 = typename TestFixture::LeafAB{eigen_vec};
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Construct a plain leaf from a non-plain leaf, whose ImplType is an Eigen sum expression
TYPED_TEST_P(HomogeneousTest, directConstructFromExpr) {
    const auto eigen_a =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Random()};
    const auto eigen_b =
      typename TestFixture::EigenVector4{TestFixture::EigenVector4::Random()};

    using ExprLeaf = typename TestFixture::template FramedPt<
      typename wave::internal::traits<typename TestFixture::Leaf>::template rebind<
        decltype(eigen_a + eigen_b)>,
      typename TestFixture::FrameA,
      typename TestFixture::FrameB>;
    const auto t1 = ExprLeaf{eigen_a + eigen_b};
    const typename TestFixture::LeafAB t2{t1};
    EXPECT_APPROX((eigen_a + eigen_b).eval(), t2.value());
}

// Check that the result of Random() is the same type
TYPED_TEST_P(HomogeneousTest, constructRandom) {
    auto t1 = typename TestFixture::LeafAB{};
    auto t2 = TestFixture::LeafAB::Random();
    static_assert(std::is_same<decltype(t1), decltype(t2)>{},
                  "Random() returns non-plain type");
}

TYPED_TEST_P(HomogeneousTest, deperturbExpr) {
    const auto t1 = TestFixture::LeafAB::Random();
    const auto t2 = TestFixture::LeafAC::Random();
    const auto result = typename TestFixture::VectorACB{deperturb(t1, t2)};
    const auto eigen_result =
      typename TestFixture::EigenVector3{(t1.value() - t2.value()).template head<3>()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, deperturb(t1, t2), t1, t2);
}


// When adding a test it must also be added to the REGISTER_TYPED_TEST_CASE_P call below.
// Yes, it's redundant; apparently the drawback of using type-parameterized tests.
REGISTER_TYPED_TEST_CASE_P(HomogeneousTest,
                           constructFromVector,
                           constructFromTempVector,
                           constructFromEigenExpression,
                           constructFromCoeffs,
                           copyConstructBraces,
                           copyConstructEquals,
                           copyAssign,
                           moveAssign,
                           directConstructFromExpr,
                           constructRandom,
                           deperturbExpr);
