#include "wave/geometry/geometry.hpp"
#include "test.hpp"

/** Test operations of a rigid transform object, both with and without frames.
 *
 * This test covers only RT-specific functions such as constructors. Lie group operations
 * are tested in transform_test.cpp
 */

template <typename Params>
class RigidTransformTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    using Leaf = typename Params::Leaf;

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
    using TransformM = wave::MatrixRigidTransform<Eigen::Matrix<Scalar, 4, 4>>;
    using TransformQ = wave::CompactRigidTransform<Quaternion, Vector3>;
    using RelativeRotation = wave::RelativeRotation<Vector3>;
    using Translation = wave::Translation<Vector3>;
    using RotationM = wave::MatrixRotation<Matrix3>;

    // Convenience framed types
    template <typename T, typename... F>
    using Framed = typename Params::template Framed<T, F...>;

    using LeafAA = Framed<Leaf, FrameA, FrameA>;
    using LeafAB = Framed<Leaf, FrameA, FrameB>;
    using LeafBA = Framed<Leaf, FrameB, FrameA>;
    using LeafBC = Framed<Leaf, FrameB, FrameC>;
    using LeafAC = Framed<Leaf, FrameA, FrameC>;
    using RotMAB = Framed<RotationM, FrameA, FrameB>;
    using RelAAB = Framed<RelativeRotation, FrameA, FrameA, FrameB>;
    using PointAAB = Framed<Translation, FrameA, FrameA, FrameB>;
    using PointAAC = Framed<Translation, FrameA, FrameA, FrameC>;
    using PointBBC = Framed<Translation, FrameB, FrameB, FrameC>;
    using TransformM_BC = Framed<TransformM, FrameB, FrameC>;
    using TransformQ_BC = Framed<TransformQ, FrameB, FrameC>;

    // Convenience random values
    const Matrix3 R1{wave::randomQuaternion<Scalar>()};
    const Vector3 t1{Vector3::Random()};

    // Static traits checks
    TICK_TRAIT_CHECK(wave::internal::is_leaf_expression<LeafAB>);
};

// Use a type-parametrized test case, meaning we can instantiate it with types later
// without knowing the types in advance.
// See https://github.com/google/googletest/blob/master/googletest/docs/advanced.md
TYPED_TEST_CASE_P(RigidTransformTest);

TYPED_TEST_P(RigidTransformTest, constructFromMatrixAndVector) {
    auto rt = typename TestFixture::LeafAB{this->R1, this->t1};

    EXPECT_APPROX(this->R1, typename TestFixture::Matrix3{rt.rotation().eval().value()});
    EXPECT_APPROX(this->t1, rt.translation().eval().value());
}

TYPED_TEST_P(RigidTransformTest, getters) {
    const auto rt = typename TestFixture::LeafAB{this->R1, this->t1};

    const auto &R = rt.rotation().eval().value();
    const auto &t = rt.translation().eval().value();

    EXPECT_APPROX(this->R1, typename TestFixture::Matrix3{R});
    EXPECT_APPROX(this->t1, t);
    //    CHECK_JACOBIANS(true, rt.rotation(), rt);
    //    CHECK_JACOBIANS(true, rt.translation(), rt);
}

TYPED_TEST_P(RigidTransformTest, assignRvalueViaSubobject) {
    auto rt = typename TestFixture::LeafAB{this->R1, this->t1};
    const auto r = TestFixture::RotMAB::Random();

    rt.translation() = typename TestFixture::PointAAB{1., 2., 3.};
    rt.rotation() = typename TestFixture::RotMAB{r};
    EXPECT_APPROX(typename TestFixture::PointAAB(1., 2., 3.), rt.translation());
    EXPECT_APPROX(r, rt.rotation());
}

TYPED_TEST_P(RigidTransformTest, assignLvalueViaSubobject) {
    auto rt = typename TestFixture::LeafAB{this->R1, this->t1};
    const auto t = typename TestFixture::PointAAB{1., 2., 3.};
    const auto r = TestFixture::RotMAB::Random();
    rt.translation() = t;
    rt.rotation() = r;
    EXPECT_APPROX(t, rt.translation());
    EXPECT_APPROX(r, rt.rotation());
}

TYPED_TEST_P(RigidTransformTest, assignViaSubobjectNoFrame) {
    // Special case where the object is Framed but has NoFrame descriptors - should be
    // assignable from unframed
    auto rt = wave::Framed<typename TestFixture::Leaf, wave::NoFrame, wave::NoFrame>{
      this->R1, this->t1};

    rt.translation() = typename TestFixture::Translation{1., 2., 3.};
    rt.rotation() = TestFixture::RotationM::Random();
    EXPECT_APPROX(typename TestFixture::Translation(1., 2., 3.), rt.translation());
}

// These tests ensure there are no problems with invalid references to temporaries, etc
TYPED_TEST_P(RigidTransformTest, sumOfProductOfSubobjects) {
    const auto rt1 = TestFixture::LeafAB::Random();
    const auto rt2 = TestFixture::LeafBC::Random();

    const auto eigen_result =
      (rt1.rotation().eval().value() * rt2.translation().eval().value() +
       rt1.translation().eval().value())
        .eval();
    const auto result = (rt1.rotation() * rt2.translation() + rt1.translation()).eval();

    EXPECT_APPROX(eigen_result, result.value());
}


TYPED_TEST_P(RigidTransformTest, copyConstruct) {
    const auto rt1 = TestFixture::LeafAB::Random();
    const auto rt2 = rt1;
    EXPECT_APPROX(rt1.rotation(), rt2.rotation());
    EXPECT_APPROX(rt1.translation(), rt2.translation());
    EXPECT_APPROX(rt1, rt2);
}


TYPED_TEST_P(RigidTransformTest, identityExpr) {
    const auto rt = TestFixture::LeafAB::Identity();

    EXPECT_TRUE(typename TestFixture::Matrix3{rt.rotation().value()}.isIdentity());
    EXPECT_TRUE(rt.translation().eval().value().isZero());
}

TYPED_TEST_P(RigidTransformTest, inverseExpr) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = typename TestFixture::LeafBA{inverse(r1)};

    const auto eigen_rotation =
      typename TestFixture::Matrix3{r1.rotation().eval().value()};
    const auto eigen_translation =
      typename TestFixture::Vector3{r1.translation().eval().value()};

    const auto expected = typename TestFixture::LeafBA{
      eigen_rotation.inverse(), -eigen_rotation.inverse() * eigen_translation};

    EXPECT_APPROX(expected, r2);
    CHECK_JACOBIANS(true, inverse(r1), r1);
}

TYPED_TEST_P(RigidTransformTest, composeWithM) {
    const auto lhs = TestFixture::LeafAB::Random();
    const auto rhs = TestFixture::TransformM_BC::Random();

    const auto result = typename TestFixture::LeafAC{lhs * rhs};

    auto expected = typename TestFixture::LeafAC{};
    expected.rotation() = lhs.rotation() * rhs.rotation();
    expected.translation() = lhs.rotation() * rhs.translation() + lhs.translation();

    EXPECT_APPROX(expected, result);
    const bool expected_unique =
      TestFixture::IsFramed ||
      !std::is_same<typename TestFixture::Leaf, typename TestFixture::TransformM>{};
    CHECK_JACOBIANS(expected_unique, lhs * rhs, lhs, rhs);
}

TYPED_TEST_P(RigidTransformTest, composeWithQ) {
    const auto lhs = TestFixture::LeafAB::Random();
    const auto rhs = TestFixture::TransformQ_BC::Random();

    const auto result = typename TestFixture::LeafAC{lhs * rhs};

    auto expected = typename TestFixture::LeafAC{};
    expected.rotation() = lhs.rotation() * rhs.rotation();
    expected.translation() = lhs.rotation() * rhs.translation() + lhs.translation();

    EXPECT_APPROX(expected, result);
    const bool expected_unique =
      TestFixture::IsFramed ||
      !std::is_same<typename TestFixture::Leaf, typename TestFixture::TransformQ>{};
    CHECK_JACOBIANS(expected_unique, lhs * rhs, lhs, rhs);
}

TYPED_TEST_P(RigidTransformTest, transformVector) {
    const auto rt = TestFixture::LeafBA::Random();
    const auto p1 = TestFixture::PointAAC::Random();
    const auto p2 = typename TestFixture::PointBBC{rt * p1};


    const auto eigen_result =
      (rt.rotation().eval().value() * p1.eval().value() + rt.translation().eval().value())
        .eval();
    EXPECT_APPROX(eigen_result, p2.eval().value());

    CHECK_JACOBIANS(true, rt * p1, rt, p1);
}

// When adding a test it must also be added to the REGISTER_TYPED_TEST_CASE_P call below.
// Yes, it's redundant; apparently the drawback of using type-parameterized tests.
REGISTER_TYPED_TEST_CASE_P(RigidTransformTest,
                           constructFromMatrixAndVector,
                           getters,
                           assignRvalueViaSubobject,
                           assignLvalueViaSubobject,
                           assignViaSubobjectNoFrame,
                           sumOfProductOfSubobjects,
                           copyConstruct,
                           identityExpr,
                           inverseExpr,
                           composeWithM,
                           composeWithQ,
                           transformVector);
