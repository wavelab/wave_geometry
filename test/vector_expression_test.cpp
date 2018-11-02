#include "wave/geometry/geometry.hpp"
#include "test.hpp"

template <typename Params>
class VectorTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    using Leaf = typename Params::Leaf;
    using Vector = typename wave::internal::traits<Leaf>::ImplType;

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

// Direct construct from an Eigen vector
TYPED_TEST_P(VectorTest, constructFromVector) {
    const auto eigen_vec = typename TestFixture::Vector{TestFixture::Vector::Random()};
    const auto t = typename TestFixture::LeafAAB{eigen_vec};
    EXPECT_APPROX(eigen_vec, t.value());
}

// Direct construct from a temporary Eigen vector
TYPED_TEST_P(VectorTest, constructFromTempVector) {
    const auto eigen_vec = typename TestFixture::Vector{TestFixture::Vector::Zero()};
    const auto t = typename TestFixture::LeafAAB{
      typename TestFixture::Vector{TestFixture::Vector::Zero()}};
    EXPECT_APPROX(eigen_vec, t.value());
}

// Direct construct from a non-plain Eigen expression
TYPED_TEST_P(VectorTest, constructFromEigenExpression) {
    const auto eigen_vec = typename TestFixture::Vector{TestFixture::Vector::Random()};
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
TYPED_TEST_P(VectorTest, constructFromCoeffs) {
    testConstructFromScalars<typename TestFixture::LeafAAB>();
}

// Construct from an object of the same type
TYPED_TEST_P(VectorTest, copyConstructBraces) {
    const auto eigen_vec = typename TestFixture::Vector{TestFixture::Vector::Random()};
    const typename TestFixture::LeafAAB t1{eigen_vec};

    const typename TestFixture::LeafAAB t2{t1};
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Construct from an object of the same type
TYPED_TEST_P(VectorTest, copyConstructEquals) {
    const auto eigen_vec = typename TestFixture::Vector{TestFixture::Vector::Random()};
    const typename TestFixture::LeafAAB t1{eigen_vec};

    const typename TestFixture::LeafAAB t2 = t1;
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Assign from an object of the same type
TYPED_TEST_P(VectorTest, copyAssign) {
    const auto eigen_vec = typename TestFixture::Vector{TestFixture::Vector::Random()};
    typename TestFixture::LeafAAB t1{eigen_vec}, t2{};

    t2 = t1;
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Assign from a temporary object of the same type
TYPED_TEST_P(VectorTest, moveAssign) {
    const auto eigen_vec = typename TestFixture::Vector{TestFixture::Vector::Random()};
    typename TestFixture::LeafAAB t2;

    t2 = typename TestFixture::LeafAAB{eigen_vec};
    EXPECT_APPROX(eigen_vec, t2.value());
}

// Construct a plain leaf from a non-plain leaf, whose ImplType is an Eigen sum expression
TYPED_TEST_P(VectorTest, directConstructFromExpr) {
    const auto eigen_a = typename TestFixture::Vector{TestFixture::Vector::Random()};
    const auto eigen_b = typename TestFixture::Vector{TestFixture::Vector::Random()};

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
TYPED_TEST_P(VectorTest, constructRandom) {
    auto t1 = typename TestFixture::LeafAAB{};
    auto t2 = TestFixture::LeafAAB::Random();
    static_assert(std::is_same<decltype(t1), decltype(t2)>{},
                  "Random() returns non-plain type");
}

TYPED_TEST_P(VectorTest, add) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();

    const auto result = typename TestFixture::LeafAAC{t1 + t2};
    const auto eigen_result = typename TestFixture::Vector{t1.value() + t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 + t2, t1, t2);
}

TYPED_TEST_P(VectorTest, addFlipped) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();

    const auto result = typename TestFixture::LeafAAC{t2 + t1};
    const auto eigen_result = typename TestFixture::Vector{t1.value() + t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t2 + t1, t2, t1);
}

TYPED_TEST_P(VectorTest, addWithFrameCast) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto expr = wave::frame_cast<typename TestFixture::FrameA,
                                       typename TestFixture::FrameC,
                                       typename TestFixture::FrameA>(
      t1 + wave::frame_cast<typename TestFixture::FrameA,
                            typename TestFixture::FrameB,
                            typename TestFixture::FrameC>(t1));

    const auto result = typename TestFixture::LeafACA{expr};
    const auto eigen_result = typename TestFixture::Vector{t1.value() * 2};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(false, expr, t1);
}

TYPED_TEST_P(VectorTest, subtract) {
    const auto t1 = TestFixture::LeafAAC::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = typename TestFixture::LeafAAB{t1 - t2};
    const auto eigen_result = typename TestFixture::Vector{t1.value() - t2.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 - t2, t1, t2);
}

TYPED_TEST_P(VectorTest, negate) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto result = typename TestFixture::LeafABA{-t1};
    const auto eigen_result = typename TestFixture::Vector{-(t1.value())};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(true, -t1, t1);
}

TYPED_TEST_P(VectorTest, squaredNorm) {
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

TYPED_TEST_P(VectorTest, norm) {
    const auto t1 = TestFixture::LeafAAB::Random();

    // Note .norm() returns an expression, which here is implicitly converted to a double
    const double result = t1.norm();
    const auto eigen_result = t1.value().norm();

    ASSERT_DOUBLE_EQ(eigen_result, result);
    CHECK_JACOBIANS(true, t1.norm(), t1);
}

TYPED_TEST_P(VectorTest, normalize) {
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

TYPED_TEST_P(VectorTest, addMultiple) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto t3 = TestFixture::LeafACA::Random();

    const auto result = typename TestFixture::LeafAAB{t1 + t2 + t3 + t1};
    const auto eigen_result =
      typename TestFixture::Vector{t1.value() + t2.value() + t3.value() + t1.value()};
    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(false, t1 + t2 + t3 + t1, t1, t2, t3);
}

TYPED_TEST_P(VectorTest, addTemporary) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto v2 = typename TestFixture::Vector{TestFixture::Vector::Random()};
    const auto result =
      typename TestFixture::LeafAAC{t1 + typename TestFixture::LeafABC{v2}};
    const auto eigen_result = typename TestFixture::Vector{t1.value() + v2};
    EXPECT_APPROX(eigen_result, result.value());
}

TYPED_TEST_P(VectorTest, constructFromZeroLeaf) {
    const auto t1 = typename TestFixture::LeafAAB{typename TestFixture::ZeroLeafAAB{}};
    const auto expected = typename TestFixture::Vector{TestFixture::Vector::Zero()};
    EXPECT_APPROX(expected, t1.value());
}

TYPED_TEST_P(VectorTest, addZeroLeaf) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = typename TestFixture::ZeroLeafABC{};
    const auto result = (t1 + t2).eval();
    EXPECT_APPROX(t1.value(), result.value());

    const auto result_free_fn = eval(t1 + t2);
    EXPECT_APPROX(t1.value(), result_free_fn.value());
    CHECK_JACOBIANS(true, t1 + t2, t1, t2);
}

TYPED_TEST_P(VectorTest, addZeroLeafLhs) {
    const auto t1 = typename TestFixture::ZeroLeafAAB{};
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = (t1 + t2).eval();
    EXPECT_APPROX(t2.value(), result.value());
    CHECK_JACOBIANS(true, t1 + t2, t1, t2);
}

TYPED_TEST_P(VectorTest, addTwoZeroLeafs) {
    const auto t1 = typename TestFixture::ZeroLeafAAB{};
    const auto t2 = typename TestFixture::ZeroLeafABC{};
    const auto result = (t1 + t2).eval();

    const auto expected = typename TestFixture::LeafAAC{TestFixture::Vector::Zero()};
    EXPECT_APPROX(expected, result);
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 + t2, t1, t2);
}

TYPED_TEST_P(VectorTest, scaleLeft) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto a = wave::uniformRandom<typename TestFixture::Scalar>(-3.0, 3.0);
    const auto result = typename TestFixture::LeafAAB{a * t1};
    const auto eigen_result = typename TestFixture::Vector{a * t1.value()};
    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(true, a * t1, a, t1);
}

TYPED_TEST_P(VectorTest, scaleRight) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto a = wave::uniformRandom<typename TestFixture::Scalar>(-3.0, 3.0);
    const auto result = typename TestFixture::LeafAAB{t1 * a};
    const auto eigen_result = typename TestFixture::Vector{t1.value() * a};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(true, t1 * a, t1, a);
}

TYPED_TEST_P(VectorTest, scaleDivideRight) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto a = wave::uniformRandom<typename TestFixture::Scalar>(-3.0, 3.0);
    const auto result = typename TestFixture::LeafAAB{t1 / a};
    const auto eigen_result = typename TestFixture::Vector{t1.value() / a};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(true, t1 * a, t1, a);
}

TYPED_TEST_P(VectorTest, scaleByExpressionLeft) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = typename TestFixture::LeafAAB{t2.norm() * t1};
    const auto eigen_result =
      typename TestFixture::Vector{t2.value().norm() * t1.value()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t2.norm() * t1, t2, t1);
}

TYPED_TEST_P(VectorTest, scaleByExpressionRight) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = typename TestFixture::LeafAAB{t1 * t2.norm()};
    const auto eigen_result =
      typename TestFixture::Vector{t1.value() * t2.value().norm()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 * t2.norm(), t1, t2);
}

TYPED_TEST_P(VectorTest, scaleDivideByExpressionRight) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto t2 = TestFixture::LeafABC::Random();
    const auto result = typename TestFixture::LeafAAB{t1 / t2.norm()};
    const auto eigen_result =
      typename TestFixture::Vector{t1.value() / t2.value().norm()};

    EXPECT_APPROX(eigen_result, result.value());
    CHECK_JACOBIANS(TestFixture::IsFramed, t1 / t2.norm(), t1, t2);
}

// When adding a test it must also be added to the REGISTER_TYPED_TEST_CASE_P call below.
// Yes, it's redundant; apparently the drawback of using type-parameterized tests.
REGISTER_TYPED_TEST_CASE_P(VectorTest,
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
                           add,
                           addFlipped,
                           addWithFrameCast,
                           subtract,
                           negate,
                           squaredNorm,
                           norm,
                           normalize,
                           dotProduct,
                           addNorms,
                           addMultiple,
                           addTemporary,
                           constructFromZeroLeaf,
                           addZeroLeaf,
                           addZeroLeafLhs,
                           addTwoZeroLeafs,
                           scaleLeft,
                           scaleRight,
                           scaleDivideRight,
                           scaleByExpressionLeft,
                           scaleByExpressionRight,
                           scaleDivideByExpressionRight);
