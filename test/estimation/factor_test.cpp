/**
 * @file
 * @author lkoppel
 * Tests for Factor class template.
 *
 * Note that Factors and their methods tested here (evaluate(), evaluateWithJacobians())
 * are not for direct use by the user.
 */

#include "test_factors.hpp"


template <typename Params>
class FactorTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    using Leaf = typename Params::Leaf;
    using Translation = wave::Translation<Eigen::Matrix<Scalar, 3, 1>>;
    using RotQ = wave::QuaternionRotation<Eigen::Quaternion<Scalar>>;
    using RelRot = wave::RelativeRotation<Eigen::Matrix<Scalar, 3, 1>>;

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
    using RelRotBC = Framed<RelRot, FrameB, FrameC>;
    using TransAAB = Framed<Translation, FrameA, FrameA, FrameB>;
    using TransBAB = Framed<Translation, FrameB, FrameA, FrameB>;
    using TransCAB = Framed<Translation, FrameC, FrameA, FrameB>;
    using TransBBC = Framed<Translation, FrameB, FrameB, FrameC>;
    using TransAAC = Framed<Translation, FrameA, FrameA, FrameC>;

    using TangentAA = typename wave::internal::traits<LeafAA>::TangentType;
    using Identity = wave::internal::identity_t<LeafAA>;
    using Block = wave::BlockMatrix<LeafAA, LeafAA>;


    // Static traits checks
    TICK_TRAIT_CHECK(
      wave::internal::is_compound_vector_expression<example::RangeBearingd>);

    using BT = wave::CompoundBoxMinus<
      example::RangeBearing<wave::Scalar<double>, wave::Scalar<double>>,
      example::RangeBearing<wave::Scalar<double>, wave::Scalar<double>>>;
    using PT = wave::internal::plain_output_t<BT>;
    TICK_TRAIT_CHECK(wave::internal::is_compound_leaf_expression<example::RangeBearingd>);
};

// The list of implementation types to run each test case on
using LeafTypes = test_types_list<wave::RigidTransformMd>;

// The following tests will be built for each type in LeafTypes
TYPED_TEST_CASE(FactorTest, LeafTypes);


TYPED_TEST(FactorTest, evaluateDistanceFactor) {
    using TranslationVar = wave::FactorVariable<typename TestFixture::TransAAB>;

    const auto measured_dist = 1.23;
    const auto stddev = 0.1;

    const auto meas = wave::Uncertain<wave::Scalar<double>, wave::DiagonalNoise>{
      measured_dist, wave::DiagonalNoise<wave::Scalar<double>>::FromStdDev(stddev)};

    const auto f = wave::makeFactor<example::DistanceFunctor>(
      meas, std::make_shared<TranslationVar>(), std::make_shared<TranslationVar>());

    // Prepare test inputs
    const auto param_a = typename TestFixture::TransAAB{1.1, 2.2, 3.3};
    const auto param_b = typename TestFixture::TransAAB{1.0, 3.0, 2.3};

    // Calculate expected values (use analytic jacobians)
    const auto dist = Eigen::Vector3d{param_a.value() - param_b.value()}.norm();

    // Test just the functor first
    auto dist_expr = example::DistanceFunctor{}(param_a, param_b);
    EXPECT_DOUBLE_EQ(dist, dist_expr);
    CHECK_JACOBIANS(false, dist_expr, param_a, param_b);

    // The expected residual is normalized
    const auto expected_residual = (dist - measured_dist) / stddev;

    // Call and compare
    const auto residual_vec = f.evaluate(param_a, param_b);
    ASSERT_EQ(1, residual_vec.SizeAtCompileTime);
    EXPECT_DOUBLE_EQ(expected_residual, residual_vec[0]);


    // Evaluate with Jacobians
    auto J1 = wave::BlockMatrix<wave::Scalar<double>, wave::Translationd>{};
    auto J2 = wave::BlockMatrix<wave::Scalar<double>, wave::Translationd>{};
    auto j_residual_vec = Eigen::Matrix<double, 1, 1>{};
    std::tie(j_residual_vec, J1, J2) = f.evaluateWithJacobians(param_a, param_b);

    // Make sure results of evaluateWithJacobians() match results of evaluate()
    ASSERT_EQ(1, j_residual_vec.SizeAtCompileTime);
    EXPECT_DOUBLE_EQ(expected_residual, j_residual_vec[0]);
    EXPECT_APPROX(J1, (param_a - param_b).norm().jacobian(param_a));
    EXPECT_APPROX(J2, (param_a - param_b).norm().jacobian(param_b));
}

TYPED_TEST(FactorTest, evaluateRangeBearingFactor) {
    using PoseVar = wave::FactorVariable<typename TestFixture::LeafAB>;
    using TranslationVar = wave::FactorVariable<typename TestFixture::TransAAC>;

    // Make up a bearing starting with a random small rotation perpendicular to Z axis
    // The bearing here is the magnitude of rotation between the Z axis of frame "B" and a
    // vector to the landmark (frame "C")
    auto rr = typename TestFixture::RelRotBC{
      wave::uniformRandom<double>(0., 1.), wave::uniformRandom<double>(0., 1.), 0.};
    const auto rot = exp(rr).eval();
    const double actual_dist = 10.3;
    const auto vector_to_landmark =
      typename TestFixture::TransBBC{rot.value() * Eigen::Vector3d{0., 0., actual_dist}};
    const double actual_bearing = Eigen::AngleAxisd{rot.value()}.angle();
    const double measured_bearing = actual_bearing - 0.01;
    const double measured_dist = 10.5;

    const auto meas = wave::Uncertain<example::RangeBearingd, wave::DiagonalNoise>{
      example::RangeBearingd{measured_dist, measured_bearing},
      wave::DiagonalNoise<example::RangeBearingd>::FromStdDev(0.1, 0.2)};

    // Prepare test inputs
    auto actual_pose = TestFixture::LeafAB::Random();
    auto actual_landmark_pos =
      typename TestFixture::TransAAC{actual_pose * vector_to_landmark};

    // Test just the functor first
    const auto tol = 1e-6;
    auto rb_expr = example::RangeBearingFunctor{}(actual_pose, actual_landmark_pos);
    EXPECT_NEAR(actual_dist, rb_expr.range(), tol);
    EXPECT_NEAR(actual_bearing, rb_expr.bearing(), tol);
    // Check the Jacobians against numdiff
    // Note expression is not a guaranteed-unique tree since actual_landmark_pos appears
    // more than once
    CHECK_JACOBIANS(false, rb_expr, actual_pose, actual_landmark_pos);


    // Test the factor. Note the residual is normalized.
    const auto f = wave::makeFactor<example::RangeBearingFunctor>(
      meas, std::make_shared<PoseVar>(), std::make_shared<TranslationVar>());
    const auto residual_vec = f.evaluate(actual_pose, actual_landmark_pos);
    ASSERT_EQ(2, residual_vec.SizeAtCompileTime);
    EXPECT_NEAR((actual_dist - measured_dist) / 0.1, residual_vec[0], tol);
    EXPECT_NEAR((actual_bearing - measured_bearing) / 0.2, residual_vec[1], tol);

    // Evaluate with Jacobians
    auto J1 = wave::BlockMatrix<example::RangeBearingd, typename TestFixture::LeafAB>{};
    auto J2 = wave::BlockMatrix<example::RangeBearingd, typename TestFixture::TransAAC>{};
    auto j_residual_vec = Eigen::Vector2d{};
    std::tie(j_residual_vec, J1, J2) =
      f.evaluateWithJacobians(actual_pose, actual_landmark_pos);

    // Make sure results of evaluateWithJacobians() match results of evaluate()
    EXPECT_EQ(residual_vec, j_residual_vec);
    EXPECT_APPROX(J1, rb_expr.jacobian(actual_pose));
    EXPECT_APPROX(J2, rb_expr.jacobian(actual_landmark_pos));
}
