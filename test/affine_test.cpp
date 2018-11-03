/**
 * @file
 * @author lkoppel
 * Tests for affine objects (points and vectors)
 */

#include "wave/geometry/geometry.hpp"
#include "test.hpp"

template <typename Params>
class AffineTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    // The affine leaf (might be a point or vector)
    using Leaf = typename Params::Leaf;
    // The affine (actually Euclidean) vector that is the difference type of Leaf
    using Vector = typename wave::internal::traits<Leaf>::TangentType;
    using EigenVector = typename wave::internal::traits<Leaf>::ImplType;


    // Used for CHECK_JACOBIANS macro
    static constexpr bool IsFramed = Params::IsFramed;
    static constexpr bool LeafIsNotVector = !std::is_same<Leaf, Vector>{};
    static constexpr bool IsFramedOrDifferent = IsFramed || LeafIsNotVector;

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
    using LeafAAC = Framed<Leaf, FrameA, FrameA, FrameC>;
    using LeafBAC = Framed<Leaf, FrameB, FrameA, FrameC>;
    using VectorAAB = Framed<Vector, FrameA, FrameA, FrameB>;
    using VectorABC = Framed<Vector, FrameA, FrameB, FrameC>;
    using VectorABA = Framed<Vector, FrameA, FrameB, FrameA>;
    using VectorACA = Framed<Vector, FrameA, FrameC, FrameA>;
    using VectorAAC = Framed<Vector, FrameA, FrameA, FrameC>;
    using VectorBAC = Framed<Vector, FrameB, FrameA, FrameC>;

    using ZeroLeafAAB = wave::Zero<LeafAAB>;
    using ZeroLeafABC = wave::Zero<LeafABC>;
    using ZeroVectorAAB = wave::Zero<VectorAAB>;
    using ZeroVectorABC = wave::Zero<VectorABC>;

    // Static traits checks
    // Currently internal vector_leaf trait applies to affine points as well
    TICK_TRAIT_CHECK(wave::internal::is_vector_leaf<Leaf>);
    TICK_TRAIT_CHECK(wave::internal::is_vector_leaf<LeafAAB>);
    TICK_TRAIT_CHECK(wave::internal::is_leaf_expression<ZeroLeafAAB>);
    TICK_TRAIT_CHECK(wave::internal::is_unary_expression<
                     wave::FrameCast<FrameC, FrameB, FrameA, LeafAAB>>);
};

// Use a type-parametrized test case, meaning we can instantiate it with types later
// without knowing the types in advance.
// See https://github.com/google/googletest/blob/master/googletest/docs/advanced.md
TYPED_TEST_CASE_P(AffineTest);

// Direct construct from an Eigen vector
TYPED_TEST_P(AffineTest, constructFromVector) {
    const auto eigen_vec =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};
    const auto t = typename TestFixture::LeafAAB{eigen_vec};
    EXPECT_APPROX(eigen_vec, t.value());
}

// Direct construct from a temporary Eigen vector
TYPED_TEST_P(AffineTest, constructFromTempVector) {
    const auto eigen_vec =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Zero()};
    const auto t = typename TestFixture::LeafAAB{
      typename TestFixture::EigenVector{TestFixture::EigenVector::Zero()}};
    EXPECT_APPROX(eigen_vec, t.value());
}

// Direct construct from a non-plain Eigen expression
TYPED_TEST_P(AffineTest, constructFromEigenExpression) {
    const auto eigen_vec =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};
    const auto t = typename TestFixture::LeafAAB{-eigen_vec};
    EXPECT_APPROX((-eigen_vec).eval(), t.value());
}

// Helper so we can try T{1.0, 0, 0} for 3-vectors but not 6-vectors
template <typename T,
          std::enable_if_t<wave::internal::eval_traits<T>::Size == 3, int> = 0>
void testConstructFromScalars() {
    using S = typename wave::internal::eval_traits<T>::Scalar;
    using Vector = typename wave::internal::eval_traits<T>::ImplType;

    const auto t = T{S{1.0}, S{0}, S{0}};
    const auto eigen_vec = Vector{S{1.0}, S{0}, S{0}};
    EXPECT_APPROX(eigen_vec, t.value());
}

template <typename T,
          std::enable_if_t<wave::internal::eval_traits<T>::Size != 3, int> = 0>
void testConstructFromScalars() {}

// Direct construct from three scalars (only for 3-vector case)
TYPED_TEST_P(AffineTest, constructFromCoeffs) {
    testConstructFromScalars<typename TestFixture::LeafAAB>();
}

// Construct from an object of the same type
TYPED_TEST_P(AffineTest, copyConstructBraces) {
    const auto eigen_vec =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};
    const typename TestFixture::LeafAAB t1{eigen_vec};

    const typename TestFixture::LeafAAB t2{t1};
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Construct from an object of the same type
TYPED_TEST_P(AffineTest, copyConstructEquals) {
    const auto eigen_vec =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};
    const typename TestFixture::LeafAAB t1{eigen_vec};

    const typename TestFixture::LeafAAB t2 = t1;
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Assign from an object of the same type
TYPED_TEST_P(AffineTest, copyAssign) {
    const auto eigen_vec =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};
    typename TestFixture::LeafAAB t1{eigen_vec}, t2{};

    t2 = t1;
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Assign from a temporary object of the same type
TYPED_TEST_P(AffineTest, moveAssign) {
    const auto eigen_vec =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};
    typename TestFixture::LeafAAB t2;

    t2 = typename TestFixture::LeafAAB{eigen_vec};
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Construct a plain leaf from a non-plain leaf, whose ImplType is an Eigen sum expression
TYPED_TEST_P(AffineTest, directConstructFromExpr) {
    const auto eigen_a =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};
    const auto eigen_b =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};

    using ExprLeaf = typename TestFixture::template Framed<
      typename wave::internal::traits<typename TestFixture::Leaf>::template rebind<
        decltype(eigen_a + eigen_b)>,
      typename TestFixture::FrameA,
      typename TestFixture::FrameA,
      typename TestFixture::FrameB>;
    const auto t1 = ExprLeaf{eigen_a + eigen_b};
    const typename TestFixture::LeafAAB t2{t1};
    EXPECT_APPROX((eigen_a + eigen_b).eval(), t2.value());
}

// Check that the result of Random() is the same type
TYPED_TEST_P(AffineTest, constructRandom) {
    auto t1 = typename TestFixture::LeafAAB{};
    auto t2 = TestFixture::LeafAAB::Random();
    static_assert(std::is_same<decltype(t1), decltype(t2)>{},
                  "Random() returns non-plain type");
}

TYPED_TEST_P(AffineTest, subtract) {
    const auto t1 = TestFixture::LeafAAC::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = typename TestFixture::LeafAAB{t1 - t2};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() - t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 - t2, t1, t2);
}

TYPED_TEST_P(AffineTest, addLeafAndVector) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::VectorABC::Random();

    const auto result = typename TestFixture::LeafAAC{t1 + t2};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() + t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 + t2, t1, t2);
}

TYPED_TEST_P(AffineTest, addVectorAndLeaf) {
    const auto t1 = TestFixture::VectorAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();

    const auto result = typename TestFixture::LeafAAC{t2 + t1};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() + t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t2 + t1, t2, t1);
}

TYPED_TEST_P(AffineTest, addVectorWithFrameCast) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::VectorAAB::Random();
    const auto expr = wave::frame_cast<typename TestFixture::FrameA,
                                       typename TestFixture::FrameC,
                                       typename TestFixture::FrameA>(
      t1 + wave::frame_cast<typename TestFixture::FrameA,
                            typename TestFixture::FrameB,
                            typename TestFixture::FrameC>(t2));

    const auto result = typename TestFixture::LeafACA{expr};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() + t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(false, expr, t1);
}

TYPED_TEST_P(AffineTest, addMultiple) {
    const auto t1 = TestFixture::VectorAAB::Random();
    const auto t2 = TestFixture::VectorABC::Random();
    const auto t3 = TestFixture::LeafACA::Random();

    const auto result = typename TestFixture::LeafAAB{t1 + t2 + t3 + t1};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() + t2.value() +
                                                                t3.value() + t1.value()};
    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(false, t1 + t2 + t3 + t1, t1, t2, t3);
}

TYPED_TEST_P(AffineTest, addTemporaryVector) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto v2 = typename TestFixture::EigenVector{TestFixture::EigenVector::Random()};
    const auto result =
      typename TestFixture::LeafAAC{t1 + typename TestFixture::VectorABC{v2}};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() + v2};
    EXPECT_APPROX(eigen_result, result.value());
}

TYPED_TEST_P(AffineTest, subtractVector) {
    const auto t1 = TestFixture::LeafAAC::Random();
    const auto t2 = TestFixture::VectorABC::Random();
    const auto result = typename TestFixture::LeafAAB{t1 - t2};
    const auto eigen_result = typename TestFixture::EigenVector{t1.value() - t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 - t2, t1, t2);
}

TYPED_TEST_P(AffineTest, constructFromZeroLeaf) {
    const auto t1 = typename TestFixture::LeafAAB{typename TestFixture::ZeroLeafAAB{}};
    const auto expected =
      typename TestFixture::EigenVector{TestFixture::EigenVector::Zero()};
    EXPECT_APPROX(expected, t1.value());
}

TYPED_TEST_P(AffineTest, addVectorAndZeroLeaf) {
    const auto t1 = TestFixture::VectorAAB::Random();
    const auto t2 = typename TestFixture::ZeroLeafABC{};
    const auto result = (t1 + t2).eval();
    EXPECT_APPROX(t1.value(), result.value());
    CHECK_JACOBIANS(true, t1 + t2, t1, t2);
}

TYPED_TEST_P(AffineTest, addZeroLeafAndVector) {
    const auto t1 = typename TestFixture::ZeroLeafAAB{};
    const auto t2 = TestFixture::VectorABC::Random();
    const auto result = (t1 + t2).eval();
    EXPECT_APPROX(t2.value(), result.value());
    CHECK_JACOBIANS(true, t1 + t2, t1, t2);
}

TYPED_TEST_P(AffineTest, addZeroVectorAndLeaf) {
    const auto t1 = typename TestFixture::ZeroVectorAAB{};
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = (t1 + t2).eval();
    EXPECT_APPROX(t2.value(), result.value());
    CHECK_JACOBIANS(true, t1 + t2, t1, t2);
}

TYPED_TEST_P(AffineTest, addLeafAndZeroVector) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = typename TestFixture::ZeroVectorABC{};
    const auto result = (t1 + t2).eval();
    EXPECT_APPROX(t1.value(), result.value());
    CHECK_JACOBIANS(true, t1 + t2, t1, t2);
}

TYPED_TEST_P(AffineTest, addZeroLeafAndZeroVector) {
    const auto t1 = typename TestFixture::ZeroLeafAAB{};
    const auto t2 = typename TestFixture::ZeroVectorABC{};
    const auto result = (t1 + t2).eval();
    EXPECT_TRUE(result.value().isZero());
    EXPECT_TRUE(result.isZero());
    CHECK_JACOBIANS(TestFixture::IsFramedOrDifferent, t1 + t2, t1, t2);
}


// When adding a test it must also be added to the REGISTER_TYPED_TEST_CASE_P call below.
// Yes, it's redundant; apparently the drawback of using type-parameterized tests.
REGISTER_TYPED_TEST_CASE_P(AffineTest,
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
                           subtract,
                           addLeafAndVector,
                           addVectorAndLeaf,
                           addVectorWithFrameCast,
                           addMultiple,
                           addTemporaryVector,
                           subtractVector,
                           constructFromZeroLeaf,
                           addVectorAndZeroLeaf,
                           addZeroLeafAndVector,
                           addZeroVectorAndLeaf,
                           addLeafAndZeroVector,
                           addZeroLeafAndZeroVector);
