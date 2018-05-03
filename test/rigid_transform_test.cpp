#include "wave/geometry/geometry.hpp"
#include "test.hpp"

/** Test operations of a rigid transform object, both with and without frames.
 *
 * This test covers only RT-specific functions such as constructors. Lie group operations
 * are tested in transform_test.cpp
 */
template <typename Params>
struct D;
template <typename Params>
class RigidTransformTest : public testing::Test {
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
    using TransformM = wave::MatrixRigidTransform<Eigen::Matrix<Scalar, 4, 4>>;
    using TransformQ = wave::CompactRigidTransform<Eigen::Matrix<Scalar, 7, 1>>;
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
    using PointAAC = Framed<Translation, FrameA, FrameA, FrameC>;
    using PointBBC = Framed<Translation, FrameB, FrameB, FrameC>;
    using TransformM_BC = Framed<TransformM, FrameB, FrameC>;
    using TransformQ_BC = Framed<TransformQ, FrameB, FrameC>;

    // Convenience random values
    const Matrix3 R1{wave::randomQuaternion<Scalar>()};
    const Vector3 t1{Vector3::Random()};

    // Static traits checks
    //    TICK_TRAIT_CHECK(wave::internal::is_rt_leaf<LeafAB>);
};

// The list of implementation types to run each test case on
using LeafTypes = test_types_list<wave::RigidTransformMd, wave::RigidTransformQd>;

// The following tests will be built for each type in LeafTypes
TYPED_TEST_CASE(RigidTransformTest, LeafTypes);

TYPED_TEST(RigidTransformTest, constructFromMatrixAndVector) {
    auto rt = typename TestFixture::LeafAB{this->R1, this->t1};

    EXPECT_APPROX(this->R1, typename TestFixture::Matrix3{rt.rotation().value()});
    EXPECT_APPROX(this->t1, rt.translation().value());
}

TYPED_TEST(RigidTransformTest, getters) {
    const auto rt = typename TestFixture::LeafAB{this->R1, this->t1};

    const auto &R = rt.rotation().value();
    const auto &t = rt.translation().value();

    EXPECT_APPROX(this->R1, typename TestFixture::Matrix3{R});
    EXPECT_APPROX(this->t1, t);
}

// These tests ensure there are no problems with invalid references to temporaries, etc
TYPED_TEST(RigidTransformTest, sumOfProductOfSubobjects) {
    const auto rt1 = TestFixture::LeafAB::Random();
    const auto rt2 = TestFixture::LeafBC::Random();

    const auto eigen_result =
      (rt1.rotation().value() * rt2.translation().value() + rt1.translation().value())
        .eval();
    const auto result = (rt1.rotation() * rt2.translation() + rt1.translation()).eval();

    EXPECT_APPROX(eigen_result, result.value());
}


TYPED_TEST(RigidTransformTest, copyConstruct) {
    const auto rt1 = TestFixture::LeafAB::Random();
    const auto rt2 = rt1;
    EXPECT_APPROX(rt1.rotation(), rt2.rotation());
    EXPECT_APPROX(rt1.translation(), rt2.translation());
    EXPECT_APPROX(rt1, rt2);
}


TYPED_TEST(RigidTransformTest, identity) {
    const auto rt = TestFixture::LeafAB::Identity();

    EXPECT_TRUE(typename TestFixture::Matrix3{rt.rotation().value()}.isIdentity());
    EXPECT_TRUE(rt.translation().value().isZero());
}

TYPED_TEST(RigidTransformTest, inverse) {
    const auto r1 = TestFixture::LeafAB::Random();
    const auto r2 = typename TestFixture::LeafBA{inverse(r1)};

    const auto eigen_rotation = typename TestFixture::Matrix3{r1.rotation().value()};
    const auto eigen_translation =
      typename TestFixture::Vector3{r1.translation().value()};

    const auto expected = typename TestFixture::LeafBA{
      eigen_rotation.inverse(), -eigen_rotation.inverse() * eigen_translation};

    EXPECT_APPROX(expected, r2);
    CHECK_JACOBIANS(true, inverse(r1), r1);
}

TYPED_TEST(RigidTransformTest, composeWithM) {
    const auto lhs = TestFixture::LeafAB::Random();
    const auto rhs = TestFixture::TransformM_BC::Random();

    const auto result = typename TestFixture::LeafAC{lhs * rhs};

    auto expected = typename TestFixture::LeafAC{};
    expected.rotation().value() = lhs.rotation().value() * rhs.rotation().value();
    expected.translation().value() =
      lhs.rotation().value() * rhs.translation().value() + lhs.translation().value();

    EXPECT_APPROX(expected, result);
    const bool expected_unique =
      TestFixture::IsFramed ||
      !std::is_same<typename TestFixture::Leaf, typename TestFixture::TransformM>{};
    CHECK_JACOBIANS(expected_unique, lhs * rhs, lhs, rhs);
}

TYPED_TEST(RigidTransformTest, composeWithQ) {
    const auto lhs = TestFixture::LeafAB::Random();
    const auto rhs = TestFixture::TransformQ_BC::Random();

    const auto result = typename TestFixture::LeafAC{lhs * rhs};

    auto expected = typename TestFixture::LeafAC{};
    expected.rotation().value() = lhs.rotation().value() * rhs.rotation().value();
    expected.translation().value() =
      lhs.rotation().value() * rhs.translation().value() + lhs.translation().value();

    EXPECT_APPROX(expected, result);
    const bool expected_unique =
      TestFixture::IsFramed ||
      !std::is_same<typename TestFixture::Leaf, typename TestFixture::TransformQ>{};
    CHECK_JACOBIANS(expected_unique, lhs * rhs, lhs, rhs);
}

TYPED_TEST(RigidTransformTest, transformVector) {
    const auto rt = TestFixture::LeafBA::Random();
    const auto p1 = TestFixture::PointAAC::Random();
    const auto p2 = typename TestFixture::PointBBC{rt * p1};


    const auto eigen_result =
      (rt.rotation().value() * p1.value() + rt.translation().value()).eval();
    EXPECT_APPROX(eigen_result, p2.value());

    CHECK_JACOBIANS(true, rt * p1, rt, p1);
}
