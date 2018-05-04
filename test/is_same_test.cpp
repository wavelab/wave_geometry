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

TEST(IsSame, complex) {
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
