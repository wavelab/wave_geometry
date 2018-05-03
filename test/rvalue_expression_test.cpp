#include "wave/geometry/geometry.hpp"
#include "test.hpp"

/** The list of implementation types to run each test case on */
using LeafTypes = testing::Types<UnframedParams<wave::Translationd>>;

template <typename Params>
class RvalueTest : public testing::Test {
 protected:
    using Leaf = typename Params::Leaf;
    using Vector = typename wave::internal::traits<Leaf>::ImplType;

    using FrameA = typename Params::FrameA;
    using FrameB = typename Params::FrameB;
    using FrameC = typename Params::FrameC;

    // Convenience framed types
    template <typename T, typename... F>
    using Framed = typename Params::template Framed<T, F...>;

    using LeafAAA = Framed<Leaf, FrameA, FrameA, FrameA>;
    using LeafAAB = Framed<Leaf, FrameA, FrameA, FrameB>;
    using LeafABC = Framed<Leaf, FrameA, FrameB, FrameC>;
    using LeafABA = Framed<Leaf, FrameA, FrameB, FrameA>;
    using LeafACA = Framed<Leaf, FrameA, FrameC, FrameA>;
    using LeafAAC = Framed<Leaf, FrameA, FrameA, FrameC>;

    using ZeroLeafAAB = wave::Zero<LeafAAB>;
    using ZeroLeafABC = wave::Zero<LeafABC>;
};
TYPED_TEST_CASE(RvalueTest, LeafTypes);


TYPED_TEST(RvalueTest, negate) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto result = typename TestFixture::LeafABA{-t1};

    const auto eigen_result = typename TestFixture::Vector{-t1.value()};
    EXPECT_APPROX(eigen_result, result.value());
}

TYPED_TEST(RvalueTest, addTemporary) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto v2 = typename TestFixture::Vector{TestFixture::Vector::Random()};
    const auto result =
      typename TestFixture::LeafAAC{t1 + typename TestFixture::LeafABC{v2}};
    const auto eigen_result = typename TestFixture::Vector{t1.value() + v2};
    EXPECT_APPROX(eigen_result, result.value());
}


TYPED_TEST(RvalueTest, addTemporaryExpr) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto v2 = typename TestFixture::Vector{TestFixture::Vector::Random()};

    // Sum with rvalue
    const auto expr =
      typename TestFixture::LeafAAC{t1 + typename TestFixture::LeafABC{v2}};

    const auto result = typename TestFixture::LeafAAC{expr};

    const auto eigen_result = typename TestFixture::Vector{t1.value() + v2};
    EXPECT_APPROX(eigen_result, result.value());
}

TYPED_TEST(RvalueTest, addTemporaryExpr2) {
    const auto t1 = TestFixture::LeafAAB::Random();
    const auto v2 = typename TestFixture::Vector{TestFixture::Vector::Random()};
    const auto v3 = typename TestFixture::Vector{TestFixture::Vector::Random()};

    // Subtraction with rvalue
    const auto expr = t1 - typename TestFixture::LeafABC{v2};

    // Minus with rvalue
    const auto expr2 = expr + -(typename TestFixture::LeafACA{v3});

    const auto result = typename TestFixture::LeafAAA{expr2};

    const auto eigen_result = typename TestFixture::Vector{t1.value() - v2 - v3};
    EXPECT_APPROX(eigen_result, result.value());
}

TYPED_TEST(RvalueTest, negateWithFrameCast) {
    const auto t1 = TestFixture::LeafAAB::Random();

    const auto result =
      typename TestFixture::LeafABA{-wave::frame_cast<typename TestFixture::FrameA,
                                                      typename TestFixture::FrameA,
                                                      typename TestFixture::FrameB>(t1)};
    const auto eigen_result = typename TestFixture::Vector{-t1.value()};

    EXPECT_APPROX(eigen_result, result.value());
}