#include "wave/geometry/geometry.hpp"
#include "test.hpp"

/** Test correctness and consistency of a Rotation object, both with and without frames.
 *
 * This test covers only rotation-specific functions such as constructors and rotating
 * a vector. Lie group operations are tested in transform_test.cpp
 */
template <typename Params>
class RotationTest : public testing::Test {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using RotationM = wave::MatrixRotation<Matrix3>;
    using RotationQ = wave::QuaternionRotation<Quaternion>;
    using RotationA = wave::AngleAxisRotation<AngleAxis>;
    using RelativeRotation = wave::RelativeRotation<Vector3>;
    using Translation = wave::Translation<Vector3>;

    // Convenience framed types
    template <typename T, typename... F>
    using Framed = typename Params::template Framed<T, F...>;

    using LeafAA = Framed<Leaf, FrameA, FrameA>;
    using LeafAB = Framed<Leaf, FrameA, FrameB>;
    using LeafBA = Framed<Leaf, FrameB, FrameA>;
    using LeafBC = Framed<Leaf, FrameB, FrameC>;
    using LeafAC = Framed<Leaf, FrameA, FrameC>;
    using RelAAB = Framed<RelativeRotation, FrameA, FrameA, FrameB>;
    using PointAAB = Framed<Translation, FrameA, FrameA, FrameB>;
    using PointBAB = Framed<Translation, FrameB, FrameA, FrameB>;
    using RotationM_BC = Framed<RotationM, FrameB, FrameC>;
    using RotationQ_BC = Framed<RotationQ, FrameB, FrameC>;
    using RotationA_BC = Framed<RotationA, FrameB, FrameC>;

    // Convenience random quaternions
    const Quaternion q1 = wave::randomQuaternion<Scalar>();
    const Quaternion q2 = wave::randomQuaternion<Scalar>();

    // Static traits checks
    TICK_TRAIT_CHECK(wave::internal::is_leaf_expression<LeafAB>);
    TICK_TRAIT_CHECK(wave::internal::is_derived_rotation<LeafAB>);
    TICK_TRAIT_CHECK(wave::internal::is_binary_expression<wave::Compose<Leaf, Leaf>>);
    TICK_TRAIT_CHECK(wave::internal::is_binary_expression<wave::BoxPlus<LeafAB, RelAAB>>);
    TICK_TRAIT_CHECK(wave::internal::is_unary_expression<wave::BoxMinus<LeafAB, LeafAB>>);
};

// Use a type-parametrized test case, meaning we can instantiate it with types later
// without knowing the types in advance.
// See https://github.com/google/googletest/blob/master/googletest/docs/advanced.md
TYPED_TEST_CASE_P(RotationTest);

// Each rotation type should be constructable from all 3 Eigen rotation types
TYPED_TEST_P(RotationTest, constructFromM) {
    const typename TestFixture::Matrix3 eigen_obj{this->q1};
    const auto rotation = typename TestFixture::LeafAB{eigen_obj};
    EXPECT_APPROX(typename TestFixture::ImplType{eigen_obj}, rotation.value());
}

TYPED_TEST_P(RotationTest, constructFromQ) {
    const typename TestFixture::Quaternion eigen_obj{this->q1};
    const auto rotation = typename TestFixture::LeafAB{eigen_obj};
    EXPECT_APPROX(typename TestFixture::ImplType{eigen_obj}, rotation.value());
}


TYPED_TEST_P(RotationTest, constructFromA) {
    const typename TestFixture::AngleAxis eigen_obj{this->q1};
    const auto rotation = typename TestFixture::LeafAB{eigen_obj};
    EXPECT_APPROX(typename TestFixture::ImplType{eigen_obj}, rotation.value());
}

// Check that the result of Random() is the same type
TYPED_TEST_P(RotationTest, constructRandom) {
    auto r = TestFixture::LeafAB::Random();
    static_assert(std::is_same<typename TestFixture::LeafAB, decltype(r)>{},
                  "Random() returns non-plain type");
}

// Test methods of TransformBase
TYPED_TEST_P(RotationTest, getRotation) {
    auto r = TestFixture::LeafAB::Random();
    static_assert(
      std::is_same<typename TestFixture::LeafAB, decltype(r.rotation().eval())>{},
      "r.rotation() is not expected type");
    EXPECT_APPROX(r, r.rotation());
}

TYPED_TEST_P(RotationTest, assignViaGetter) {
    auto r = TestFixture::LeafAB::Random();
    auto r2 = TestFixture::LeafAB::Random();

    r.rotation() = r2;
    EXPECT_APPROX(r2, r);
}

// Test methods of TransformBase
TYPED_TEST_P(RotationTest, getTranslation) {
    auto r = TestFixture::LeafAB::Random();
    auto t = typename TestFixture::PointAAB{r.translation()};
    EXPECT_TRUE(t.value().isZero());
}

TYPED_TEST_P(RotationTest, rotateVector) {
    const auto r1 = TestFixture::LeafBA::Random();
    const auto p1 = TestFixture::PointAAB::Random();
    const auto p2 = typename TestFixture::PointBAB{r1 * p1};


    const auto eigen_result = (r1.value() * p1.value()).eval();
    EXPECT_APPROX(eigen_result, p2.value());

    CHECK_JACOBIANS(true, r1 * p1, r1, p1);
}

TYPED_TEST_P(RotationTest, inverseExpr) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = typename TestFixture::LeafBA{inverse(r1)};

    const auto eigen_rotation = r1.rotation().eval().value();
    const auto expected = typename TestFixture::LeafBA{eigen_rotation.inverse()};

    EXPECT_APPROX(expected, r2);
    CHECK_JACOBIANS(true, inverse(r1), r1);
}

TYPED_TEST_P(RotationTest, composeWithM) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = TestFixture::RotationM_BC::Random();
    const auto result = typename TestFixture::LeafAC{r1 * r2};

    // Convert Eigen result to our type for flexible comparison of different types
    // including non-unique quaternions
    const auto eigen_result = typename TestFixture::Matrix3{r1.value() * r2.value()};
    const auto expected = typename TestFixture::LeafAC{eigen_result};

    EXPECT_APPROX(expected, result);
    const bool expected_unique =
      TestFixture::IsFramed ||
      !std::is_same<typename TestFixture::Leaf, typename TestFixture::RotationM>{};
    CHECK_JACOBIANS(expected_unique, r1 * r2, r1, r2);
}

TYPED_TEST_P(RotationTest, composeWithQ) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = TestFixture::RotationQ_BC::Random();
    const auto result = typename TestFixture::LeafAC{r1 * r2};

    // Convert Eigen result to our type for flexible comparison of different types
    // including non-unique quaternions
    const auto eigen_result = typename TestFixture::Matrix3{r1.value() * r2.value()};
    const auto expected = typename TestFixture::LeafAC{eigen_result};

    EXPECT_APPROX(expected, result);
    const bool expected_unique =
      TestFixture::IsFramed ||
      !std::is_same<typename TestFixture::Leaf, typename TestFixture::RotationQ>{};
    CHECK_JACOBIANS(expected_unique, r1 * r2, r1, r2);
}

TYPED_TEST_P(RotationTest, composeWithA) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = TestFixture::RotationA_BC::Random();
    const auto result = typename TestFixture::LeafAC{r1 * r2};

    // Convert Eigen result to our type for flexible comparison of different types
    // including non-unique quaternions
    const auto eigen_result = typename TestFixture::Matrix3{r1.value() * r2.value()};
    const auto expected = typename TestFixture::LeafAC{eigen_result};

    EXPECT_APPROX(expected, result);
    const bool expected_unique =
      TestFixture::IsFramed ||
      !std::is_same<typename TestFixture::Leaf, typename TestFixture::RotationA>{};
    CHECK_JACOBIANS(expected_unique, r1 * r2, r1, r2);
}

// When adding a test it must also be added to the REGISTER_TYPED_TEST_CASE_P call below.
// Yes, it's redundant; apparently the drawback of using type-parameterized tests.
REGISTER_TYPED_TEST_CASE_P(RotationTest,
                           constructFromM,
                           constructFromQ,
                           constructFromA,
                           constructRandom,
                           getRotation,
                           assignViaGetter,
                           getTranslation,
                           rotateVector,
                           inverseExpr,
                           composeWithM,
                           composeWithQ,
                           composeWithA);