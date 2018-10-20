#include "wave/geometry/src/util/math/IdentityMatrix.hpp"
#include "wave/geometry/src/util/math/CrossMatrix.hpp"
#include "../test.hpp"

/** The list of implementation types to run each test case on */
using IdentityTypes = testing::Types<wave::IdentityMatrix<double, 3>>;

template <typename IdentityType_>
class IdentityExprTest : public testing::Test {
 protected:
    using IdentityType = IdentityType_;
    static constexpr int N = IdentityType::RowsAtCompileTime;
    using Scalar = typename IdentityType::Scalar;
    using MatrixType = Eigen::Matrix<Scalar, N, N>;
    using VectorType = Eigen::Matrix<Scalar, N, 1>;

    const MatrixType eye = IdentityType{};
    const MatrixType m1 = MatrixType::Random();
    const VectorType v1 = VectorType::Random();
};

TYPED_TEST_CASE(IdentityExprTest, IdentityTypes);

TYPED_TEST(IdentityExprTest, construct) {
    typename TestFixture::MatrixType expected = TestFixture::MatrixType::Identity();
    EXPECT_APPROX(expected, this->eye);
}

TYPED_TEST(IdentityExprTest, multiplySelf) {
    EXPECT_APPROX(this->eye, TypeParam{} * TypeParam{} * TypeParam{});
}

TYPED_TEST(IdentityExprTest, multiplyMatrix) {
    EXPECT_APPROX(this->m1, TypeParam{} * this->m1);
    EXPECT_APPROX(this->m1, this->m1 * TypeParam{});
}

TYPED_TEST(IdentityExprTest, multiplyVector) {
    EXPECT_APPROX(this->v1, TypeParam{} * this->v1);
    EXPECT_APPROX(this->v1.transpose(), this->v1.transpose() * TypeParam{});
}

TYPED_TEST(IdentityExprTest, multiplyInt) {
    int n = std::rand();
    typename TestFixture::MatrixType expected = this->eye * n;
    EXPECT_APPROX(expected, TypeParam{} * n);
    EXPECT_APPROX(expected, n * TypeParam{});
}

TYPED_TEST(IdentityExprTest, multiplyFloat) {
    typename TestFixture::Scalar f =
      Eigen::internal::random<typename TestFixture::Scalar>();
    typename TestFixture::MatrixType expected = this->eye * f;
    EXPECT_APPROX(expected, TypeParam{} * f);
    EXPECT_APPROX(expected, f * TypeParam{});
}

// Compile-time tests only
TYPED_TEST(IdentityExprTest, staticTests) {
    // Make sure Identity is properly being removed
    static_assert(
      wave::tmp::is_same_cr<decltype(TypeParam{} * this->v1), decltype(this->v1)>{}, "");

    static_assert(
      wave::tmp::is_same_cr<decltype(this->m1 * TypeParam{}), decltype(this->m1)>{}, "");
}


// Ensure there are no ambiguous overloaded operator issues
TEST(IdentityTest, multiplyCross) {
    using Identity3d = wave::IdentityMatrix<double, 3>;
    Eigen::Vector3d vec = Eigen::Vector3d::Random();
    Eigen::Matrix3d expected = wave::crossMatrix(vec);
    EXPECT_APPROX(expected, Identity3d{} * wave::crossMatrix(vec));
    EXPECT_APPROX(expected, wave::crossMatrix(vec) * Identity3d{});
}
