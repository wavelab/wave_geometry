#include "scalar_expression_test.hpp"

// Build tests using the ScalarTest fixture for each type in ScalarTypes
TYPED_TEST_CASE(ScalarTest, ScalarTypes);

// Direct construct from reference to a plain scalar
TYPED_TEST(ScalarTest, constructFromRef) {
    auto a = this->a();
    auto s = wave::Scalar<TypeParam>{a};
    EXPECT_EQ(a, s);
    // Note EXPECT_APPROX tests our own .isApprox() method
    EXPECT_APPROX(a, s);

    a /= 2;
    EXPECT_NE(a, s);
    EXPECT_EQ(this->a(), s);
}

// Direct construct from an rvalue
TYPED_TEST(ScalarTest, constructFromTemp) {
    const auto s = wave::Scalar<TypeParam>{this->a()};
    EXPECT_EQ(this->a(), s);
}

// Equals-construct from an rvalue
TYPED_TEST(ScalarTest, constructFromTempEquals) {
    const wave::Scalar<TypeParam> s = this->a();
    EXPECT_EQ(this->a(), s);
}

TYPED_TEST(ScalarTest, assignFromTemp) {
    auto s = wave::Scalar<TypeParam>{};
    s = s = this->a();
    EXPECT_EQ(this->a(), s);
}

// Construct from a ScalarBase expression
// (really, this should only be using the conversion operator on ScalarBase, not a special
// constructor)
TYPED_TEST(ScalarTest, directConstructFromExpression) {
    const auto t = TestFixture::Translation::Random();
    const auto s = wave::Scalar<TypeParam>{t.norm()};
    EXPECT_EQ(t.value().norm(), s);
}

// Assign from a ScalarBase expression
// (really, this should only be using the conversion operator on ScalarBase, not a special
// constructor)
TYPED_TEST(ScalarTest, assignFromExpression) {
    const auto t = TestFixture::Translation::Random();
    auto s = wave::Scalar<TypeParam>{};
    s = t.norm();
    EXPECT_EQ(t.value().norm(), s);
}

#define GENERATE_OP_TEST(OP, NAME)                          \
    TYPED_TEST(ScalarTest, NAME) {                          \
        const auto s1 = wave::Scalar<TypeParam>{this->a()}; \
        const auto s2 = wave::Scalar<TypeParam>{this->b()}; \
                                                            \
        const auto expr = s1 OP s2;                         \
                                                            \
        EXPECT_EQ(this->a() OP this->b(), eval(expr));      \
        CHECK_JACOBIANS(false, s1 OP s2, s1, s2);           \
    }

GENERATE_OP_TEST(+, add)
GENERATE_OP_TEST(-, subtract)
