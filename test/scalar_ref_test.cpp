#include "scalar_expression_test.hpp"

// Build tests using the ScalarTest fixture for each type in ScalarTypes
template <typename ScalarType_>
class RefScalarTest : public ScalarTest<ScalarType_> {};
TYPED_TEST_CASE(RefScalarTest, ScalarTypes);

// Direct construct a reference wrapper
TYPED_TEST(RefScalarTest, constructFromRef) {
    auto a = this->a();
    auto s = wave::Scalar<TypeParam &>{a};
    EXPECT_EQ(a, s);
    EXPECT_APPROX(a, s);

    a /= 2;
    EXPECT_EQ(a, s);
    EXPECT_EQ(this->a() / 2, a);
}

TYPED_TEST(RefScalarTest, copyConstructBraces) {
    auto a = this->a();
    const auto s0 = wave::Scalar<TypeParam &>{a};
    const auto s = wave::Scalar<TypeParam &>{s0};
    EXPECT_EQ(a, s);
    EXPECT_APPROX(a, s);

    a /= 2;
    EXPECT_EQ(a, s);
    EXPECT_EQ(this->a() / 2, a);
}

TYPED_TEST(RefScalarTest, copyConstructEquals) {
    auto a = this->a();
    const auto s0 = wave::Scalar<TypeParam &>{a};
    const wave::Scalar<TypeParam &> s{s0};
    EXPECT_EQ(a, s);
    EXPECT_APPROX(a, s);

    a /= 2;
    EXPECT_EQ(a, s);
    EXPECT_EQ(this->a() / 2, a);
}

TYPED_TEST(RefScalarTest, noCopyAssign) {
    // We can't copy-assign a ref-wrapper Scalar
    using R = wave::Scalar<TypeParam &>;
    static_assert(!std::is_copy_assignable<R>{}, "");
    static_assert(!std::is_move_assignable<R>{}, "");
}

TYPED_TEST(RefScalarTest, constructRegFromRef) {
    const auto a = this->a();
    const auto r = wave::Scalar<TypeParam &>{a};
    const auto s = wave::Scalar<TypeParam>{r};

    EXPECT_EQ(a, s);
}

TYPED_TEST(RefScalarTest, assignRegFromRef) {
    const auto a = this->a();
    const auto r = wave::Scalar<TypeParam &>{a};
    auto s = wave::Scalar<TypeParam>{};
    s = r;

    EXPECT_EQ(a, s);
}
