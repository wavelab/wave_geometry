#include "test.hpp"
#include "wave/geometry/dynamic.hpp"
#include "wave/geometry/geometry.hpp"

template <typename Params>
class ProxyTest : public testing::Test {
 protected:
    using Scalar = typename Params::Scalar;
    using Leaf = typename Params::Leaf;
    using ImplType = typename wave::internal::traits<Leaf>::ImplType;
    using Translation = wave::Translation<Eigen::Matrix<Scalar, 3, 1>>;


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
    using TranslationAAB = Framed<Translation, FrameA, FrameA, FrameB>;
    using TranslationBAB = Framed<Translation, FrameB, FrameA, FrameB>;
    using TranslationCAB = Framed<Translation, FrameC, FrameA, FrameB>;


    // Static traits checks

    TICK_TRAIT_CHECK(wave::internal::is_unary_expression<wave::Dynamic<LeafAB>>);
    static_assert(not wave::internal::is_leaf_expression<wave::Proxy<LeafAB>>{}, "");
    TICK_TRAIT_CHECK(wave::internal::is_stable_expression<wave::Proxy<LeafAB>>);
    static_assert(not wave::internal::is_unary_expression<wave::Proxy<LeafAB>>{}, "");
};

template <typename Params>
class RefProxyTest : public ProxyTest<Params> {};

// Run each test case for framed and unframed leaves
TYPED_TEST_CASE(ProxyTest, test_types_list<wave::RotationQd>);
TYPED_TEST_CASE(RefProxyTest, test_types_list<wave::RotationQd>);

TYPED_TEST(ProxyTest, constructDynamic) {
    const auto r = TestFixture::LeafAB::Random();
    const auto d = wave::Dynamic<typename TestFixture::LeafAB &>{r};

    // Here we evaluate the Dynamic object directly
    // Note: it's generally pointless to use Dynamic except through a DynamicBase pointer
    EXPECT_APPROX(r, d.eval());
    EXPECT_APPROX(r, eval(d));
    CHECK_JACOBIANS(true, d, r);
}

TYPED_TEST(ProxyTest, constructProxy) {
    const auto r = TestFixture::LeafAB::Random();
    // We need to construct the proxy from an rvalue
    const auto p = makeProxy(typename TestFixture::LeafAB{r});

    EXPECT_APPROX(r, p.eval());
    EXPECT_APPROX(r, eval(p));
    // Note differentiating wrt r would return zero, because we copied r
    // Just for this test, it is possible to get the target leaf
    const auto &leaf =
      dynamic_cast<const wave::Dynamic<typename TestFixture::LeafAB &&> &>(p.follow())
        .rhs();
    CHECK_JACOBIANS(false, p, leaf);
}

TYPED_TEST(ProxyTest, constructRandomProxy) {
    const wave::Proxy<typename TestFixture::LeafAB> p = TestFixture::LeafAB::Random();

    const auto &leaf =
      dynamic_cast<const wave::Dynamic<typename TestFixture::LeafAB &&> &>(p.follow())
        .rhs();
    CHECK_JACOBIANS(false, p, leaf);
}

TYPED_TEST(ProxyTest, constructComposeProxy) {
    const auto q1 = TestFixture::LeafAB::Random();
    const auto q2 = TestFixture::LeafBC::Random();
    const auto p = wave::Proxy<typename TestFixture::LeafAC>{q1 * q2};

    const auto expected = eval(q1 * q2);
    EXPECT_APPROX(expected, p.eval());
    EXPECT_APPROX(expected, eval(p));
    CHECK_JACOBIANS(false, p, q1, q2);
}

TYPED_TEST(ProxyTest, assign) {
    const auto q1 = TestFixture::TranslationAAB::Random();
    const auto q2 = TestFixture::TranslationAAB::Random();
    auto p1 = makeProxy(typename TestFixture::TranslationAAB{q1});
    auto p2 = p1;

    EXPECT_EQ(q1.value(), p1.eval().value());
    EXPECT_EQ(p1.eval().value(), p2.eval().value());

    p1 = makeProxy(typename TestFixture::TranslationAAB{q2});
    EXPECT_EQ(q2.value(), p1.eval().value());
    EXPECT_EQ(q1.value(), p2.eval().value());
    EXPECT_NE(p1.eval().value(), p2.eval().value());
}

TYPED_TEST(ProxyTest, assignExpr) {
    const auto p_A = makeProxy(TestFixture::TranslationAAB::Random());
    const auto q_CA = makeProxy(TestFixture::LeafCA::Random());
    const auto q_CA_2 = makeProxy(TestFixture::LeafCA::Random());

    auto p = makeProxy(q_CA * p_A);
    CHECK_JACOBIANS(false, p, p_A, q_CA);

    p = q_CA_2 * p_A;
    CHECK_JACOBIANS(false, p, p_A, q_CA_2);
}

TYPED_TEST(ProxyTest, assignExprNoAuto) {
    wave::Proxy<typename TestFixture::TranslationAAB> p_A =
      TestFixture::TranslationAAB::Random();
    wave::Proxy<typename TestFixture::LeafCA> q_CA = TestFixture::LeafCA::Random();
    wave::Proxy<typename TestFixture::LeafCA> q_CA_2 = TestFixture::LeafCA::Random();

    wave::Proxy<typename TestFixture::TranslationCAB> p = q_CA * p_A;
    CHECK_JACOBIANS(false, p, p_A, q_CA);

    p = q_CA_2 * p_A;
    CHECK_JACOBIANS(false, p, p_A, q_CA_2);
}

TYPED_TEST(ProxyTest, constructDeepProxyExpr) {
    auto expected = TestFixture::TranslationAAB::Random();
    auto rotations = std::vector<wave::Proxy<typename TestFixture::LeafAA>>{};
    auto expr = makeProxy(typename TestFixture::TranslationAAB{expected});
    const auto &t0 =
      static_cast<const wave::Dynamic<typename TestFixture::TranslationAAB &&> &>(
        expr.follow())
        .rhs();
    auto t1 = expr;

    for (auto i = 10; i > 0; --i) {
        rotations.emplace_back(TestFixture::LeafAA::Random());
        expr = makeProxy(rotations.back() * expr);
        expected = rotations.back() * expected;
    }

    EXPECT_APPROX(expected, expr.eval());
    EXPECT_APPROX(expected, eval(expr));
    CHECK_JACOBIANS(false, expr, t0, rotations[3], rotations[2], rotations[1]);
}

TYPED_TEST(RefProxyTest, assign) {
    const auto q1 = TestFixture::TranslationAAB::Random();
    const auto q2 = TestFixture::TranslationAAB::Random();
    const auto d1 = makeDynamic(q1);
    const auto d2 = makeDynamic(q2);
    auto p1 = ref(d1);
    auto p2 = p1;

    EXPECT_EQ(q1.value(), p1.eval().value());
    EXPECT_EQ(p1.eval().value(), p2.eval().value());

    p1 = ref(d2);
    EXPECT_EQ(q2.value(), p1.eval().value());
    EXPECT_EQ(q1.value(), p2.eval().value());
    EXPECT_NE(p1.eval().value(), p2.eval().value());
}


TYPED_TEST(RefProxyTest, differentiate) {
    auto t = TestFixture::TranslationAAB::Random();
    auto t0 = makeDynamic(t);
    auto p0 = ref(t0);
    auto t1 = makeDynamic(TestFixture::LeafAA::Random() * p0);
    auto p1 = ref(t1);
    auto t2 = makeDynamic(TestFixture::LeafAA::Random() * p1);
    auto p2 = ref(t2);
    auto t3 = makeDynamic(TestFixture::LeafAA::Random() * p2);
    auto p3 = ref(t3);

    CHECK_JACOBIANS(false, p3, p3, p2, p1, p0);
    CHECK_JACOBIANS(false, p3, t3, t2, t1, t0);

    // Getting Jacobians wrt all leaves uses the actual leaves, not proxies

    auto value_and_jac = wave::internal::evaluateWithDynamicReverseJacobians(p3);
    const auto &jac_map = value_and_jac.second;

    EXPECT_APPROX(p3.jacobian(t), Eigen::Matrix3d{jac_map.at(&t)});
    EXPECT_APPROX(p3.jacobian(t1.rhs().lhs()),
                  Eigen::Matrix3d{jac_map.at(&t1.rhs().lhs())});
    EXPECT_APPROX(p3.jacobian(t2.rhs().lhs()),
                  Eigen::Matrix3d{jac_map.at(&t2.rhs().lhs())});
    EXPECT_APPROX(p3.jacobian(t3.rhs().lhs()),
                  Eigen::Matrix3d{jac_map.at(&t3.rhs().lhs())});

    auto a = wave::internal::getLeavesMap(p3);
    for (auto &p : a) {
        EXPECT_EQ(3, p.second);
    }
}
