#include "scalar_expression_test.hpp"

// Build tests using the ScalarTest fixture for each type in ScalarTypes
template <typename ScalarType_>
class ScalarOpTest : public ScalarTest<ScalarType_> {};
TYPED_TEST_CASE(ScalarOpTest, ScalarTypes);

#define GENERATE_OP_TESTS(OP, NAME)                         \
    TYPED_TEST(ScalarOpTest, NAME) {                        \
        const auto s1 = wave::Scalar<TypeParam>{this->a()}; \
        const auto s2 = wave::Scalar<TypeParam>{this->b()}; \
                                                            \
        const auto expr = s1 OP s2;                         \
                                                            \
        EXPECT_EQ(this->a() OP this->b(), eval(expr));      \
        CHECK_JACOBIANS(false, s1 OP s2, s1, s2);           \
    }                                                       \
                                                            \
    TYPED_TEST(ScalarOpTest, NAME##ScalarOnLeft) {          \
        const auto s1 = this->a();                          \
        const auto s2 = wave::Scalar<TypeParam>{this->b()}; \
                                                            \
        const auto expr = s1 OP s2;                         \
                                                            \
        EXPECT_EQ(this->a() OP this->b(), eval(expr));      \
        CHECK_JACOBIANS(true, s1 OP s2, s1, s2);            \
    }                                                       \
    TYPED_TEST(ScalarOpTest, NAME##ScalarOnRight) {         \
        const auto s1 = wave::Scalar<TypeParam>{this->a()}; \
        const auto s2 = this->b();                          \
                                                            \
        const auto expr = s1 OP s2;                         \
                                                            \
        EXPECT_EQ(this->a() OP this->b(), eval(expr));      \
        CHECK_JACOBIANS(true, s1 OP s2, s1, s2);            \
    }


GENERATE_OP_TESTS(+, add)
GENERATE_OP_TESTS(-, subtract)
GENERATE_OP_TESTS(*, multiply)
GENERATE_OP_TESTS(/, divide)

// These tests handle variations of rvalues in multi-level expressions, and check
// Jacobians. For the other operators assume rvalues work for now. @todo generate these
// tests also?

TYPED_TEST(ScalarOpTest, addExprToScalarLeft) {
    const auto s1 = this->a();
    const auto s2 = wave::Scalar<TypeParam>{this->b()};

    const auto result = eval(s1 + s2);
    EXPECT_EQ(this->a() + this->b(), result.value());
    CHECK_JACOBIANS(true, s1 + s2, s1, s2);
}

TYPED_TEST(ScalarOpTest, addExprToScalarLeftRvalue) {
    const auto s2 = wave::Scalar<TypeParam>{this->b()};

    const auto expr = this->a() + s2;
    EXPECT_EQ(this->a() + this->b(), eval(expr));
    CHECK_JACOBIANS(false, expr, expr.lhs(), s2);
}

TYPED_TEST(ScalarOpTest, addExprToScalarLeftRvalue1) {
    const auto s1 = this->a();

    const auto expr = s1 + wave::Scalar<TypeParam>{this->b()};
    EXPECT_EQ(this->a() + this->b(), eval(expr));
    CHECK_JACOBIANS(true, expr, s1, expr.rhs());
}

TYPED_TEST(ScalarOpTest, addExprToScalarLeftRvalue2) {
    const auto expr = this->a() + wave::Scalar<TypeParam>{this->b()};
    EXPECT_EQ(this->a() + this->b(), eval(expr));
    CHECK_JACOBIANS(false, expr, expr.lhs(), expr.rhs());
}

TYPED_TEST(ScalarOpTest, addExprToScalarRight) {
    const auto s1 = this->a();
    const auto s2 = wave::Scalar<TypeParam>{this->b()};

    const auto result = eval(s2 + s1);
    EXPECT_EQ(this->b() + this->a(), result.value());
    CHECK_JACOBIANS(true, s2 + s1, s2, s1);
}

TYPED_TEST(ScalarOpTest, addExprToScalarRightRvalue) {
    const auto s2 = wave::Scalar<TypeParam>{this->b()};

    const auto expr = s2 + this->a();
    EXPECT_EQ(this->b() + this->a(), eval(expr));
    CHECK_JACOBIANS(false, expr, s2, expr.rhs());
}

TYPED_TEST(ScalarOpTest, addExprToScalarRightRvalue1) {
    const auto s1 = this->a();

    const auto expr = wave::Scalar<TypeParam>{this->b()} + s1;
    EXPECT_EQ(this->b() + this->a(), eval(expr));
    CHECK_JACOBIANS(true, expr, expr.lhs(), s1);
}

TYPED_TEST(ScalarOpTest, addExprToScalarRightRvalue2) {
    const auto expr = wave::Scalar<TypeParam>{this->b()} + this->a();
    EXPECT_EQ(this->b() + this->a(), eval(expr));
    CHECK_JACOBIANS(false, expr, expr.lhs(), expr.rhs());
}

TYPED_TEST(ScalarOpTest, subtractExprFromScalarLeft) {
    const auto s1 = this->a();
    const auto s2 = wave::Scalar<TypeParam>{this->b()};

    const auto result = eval(s1 - s2);
    EXPECT_EQ(this->a() - this->b(), result.value());
    CHECK_JACOBIANS(true, s1 - s2, s1, s2);
}

TYPED_TEST(ScalarOpTest, subtractExprFromScalarLeftRvalue) {
    const auto s2 = wave::Scalar<TypeParam>{this->b()};

    const auto expr = this->a() - s2;
    EXPECT_EQ(this->a() - this->b(), eval(expr));
    CHECK_JACOBIANS(false, expr, expr.lhs(), s2);
}

TYPED_TEST(ScalarOpTest, subtractExprFromScalarLeftRvalue1) {
    const auto s1 = this->a();

    const auto expr = s1 - wave::Scalar<TypeParam>{this->b()};
    EXPECT_EQ(this->a() - this->b(), eval(expr));
    CHECK_JACOBIANS(true, expr, s1, expr.rhs());
}

TYPED_TEST(ScalarOpTest, subtractExprFromScalarLeftRvalue2) {
    const auto expr = this->a() - wave::Scalar<TypeParam>{this->b()};
    EXPECT_EQ(this->a() - this->b(), eval(expr));
    CHECK_JACOBIANS(false, expr, expr.lhs(), expr.rhs());
}

TYPED_TEST(ScalarOpTest, subtractExprFromScalarRight) {
    const auto s1 = this->a();
    const auto s2 = wave::Scalar<TypeParam>{this->b()};

    const auto result = eval(s2 - s1);
    EXPECT_EQ(this->b() - this->a(), result.value());
    CHECK_JACOBIANS(true, s2 - s1, s2, s1);
}

TYPED_TEST(ScalarOpTest, subtractExprFromScalarRightRvalue) {
    const auto s2 = wave::Scalar<TypeParam>{this->b()};

    const auto expr = s2 - this->a();
    EXPECT_EQ(this->b() - this->a(), eval(expr));
    CHECK_JACOBIANS(false, expr, s2, expr.rhs());
}

TYPED_TEST(ScalarOpTest, subtractExprFromScalarRightRvalue1) {
    const auto s1 = this->a();

    const auto expr = wave::Scalar<TypeParam>{this->b()} - s1;
    EXPECT_EQ(this->b() - this->a(), eval(expr));
    CHECK_JACOBIANS(true, expr, expr.lhs(), s1);
}

TYPED_TEST(ScalarOpTest, subtractExprFromScalarRightRvalue2) {
    const auto expr = wave::Scalar<TypeParam>{this->b()} - this->a();
    EXPECT_EQ(this->b() - this->a(), eval(expr));
    CHECK_JACOBIANS(false, expr, expr.lhs(), expr.rhs());
}
