/**
 * @file
 *
 * Tests for comparing expression identities
 */

#include "wave/geometry/geometry.hpp"
#include "wave/geometry/debug.hpp"
#include "test.hpp"

TEST(IsSame, trivialIdentity) {
    const wave::RotationQd r1{};
    EXPECT_TRUE(wave::isSame(r1, r1));
}

TEST(IsSame, trivialFalse) {
    const wave::RotationQd r1{}, r2{};
    EXPECT_FALSE(wave::isSame(r1, r2));
}

TEST(IsSame, trivialFalseCopy) {
    const wave::RotationQd r1{};
    const auto r2 = r1;
    EXPECT_FALSE(wave::isSame(r1, r2));
}

TEST(IsSame, trivialFalseTemporary) {
    Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
    wave::RotationQd r1{};

    EXPECT_FALSE(wave::isSame(wave::RotationQd{q1}, wave::RotationQd{q1}));
}

TEST(IsSame, falseTemporaryValue) {
    // This test catches a problem with a possible implementation of isSame(): using the
    // address of the leaf's `value()`. Here Zero.value() would return a temporary, and we
    // can't take the address.
    EXPECT_FALSE(
      wave::isSame(wave::Zero<wave::Translationd>{}, wave::Zero<wave::Translationd>{}));
}

TEST(IsSame, trivialConstRef) {
    wave::RotationQd r1{};
    const auto &cr = r1;
    EXPECT_TRUE(wave::isSame(r1, cr));
}

TEST(IsSame, composition) {
    wave::RotationQd r1{};
    wave::RotationQd r2{};

    const auto comp_a = r1 * r2;
    const auto comp_b = r1 * r2;
    auto comp_a_copy = comp_a;
    EXPECT_TRUE(wave::isSame(comp_a, comp_b));
    EXPECT_TRUE(wave::isSame(comp_a, comp_a_copy));
    EXPECT_TRUE(wave::isSame(r1 * r2, r1 * r2));
    EXPECT_FALSE(wave::isSame(r1 * r2, r2 * r1));
}

TEST(IsSame, complicatedExpression) {
    wave::RotationQd r1{};
    wave::RotationQd r2{};
    wave::RotationMd r3{};

    const auto expr = r1 * inverse(r2 * inverse(r3));
    EXPECT_TRUE(
      wave::isSame(r1 * inverse(r2 * inverse(r3)), r1 * inverse(r2 * inverse(r3))));
    EXPECT_TRUE(wave::isSame(expr, r1 * inverse(r2 * inverse(r3))));

    auto expr_copy = expr;
    EXPECT_TRUE(wave::isSame(expr, expr_copy));
}

TEST(IsSame, rvalueCopy) {
    const auto t1 = wave::Translationd::Random();

    // expr is initialized with an rvalue leaf, thus it stores the leaf
    // (expr is Minus<Translationd&&>)
    const auto expr = -(wave::Translationd::Random());

    // expr2 stores a *copy* of expr, including the leaf. Therefore isSame() here is
    // false, and we can't differentiate expr2 wrt expr.rhs().
    // @todo The decision about copying expressions with stored leaves may change.
    const auto expr2 = t1 + expr;
    EXPECT_TRUE(isSame(t1, expr2.lhs()));
    EXPECT_FALSE(isSame(expr, expr2.rhs()));
    EXPECT_FALSE(isSame(expr.rhs(), expr2.rhs().rhs()));

    // Note the stored leaves are equal, though separate copies.
    EXPECT_EQ(expr.rhs().value(), expr2.rhs().rhs().value());
}

TEST(IsSame, scalarRef) {
    double a{};
    wave::Scalar<double> s0{a};
    wave::ScalarRef<double> s1{a};
    wave::ScalarRef<double> s2{a};

    EXPECT_TRUE(wave::isSame(s1, s2));
    EXPECT_FALSE(wave::isSame(s0, s1));
}

TEST(IsSame, contains_same_type) {
    using Sum = wave::Sum<wave::Scalar<double>, wave::ScalarRef<double>>;
    static_assert(wave::internal::contains_same_type<Sum, wave::ScalarRef<double>>{}, "");
    static_assert(wave::internal::contains_same_type<Sum, double>{}, "");
}
