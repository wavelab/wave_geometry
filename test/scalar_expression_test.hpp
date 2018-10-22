/**
 * @file
 * @author lkoppel
 * Fixture for scalar expression tests
 */

#ifndef WAVE_GEOMETRY_SCALAR_EXPRESSION_TEST_HPP
#define WAVE_GEOMETRY_SCALAR_EXPRESSION_TEST_HPP

#include "wave/geometry/geometry.hpp"
#include "test.hpp"

// The list of implementation types to run each test case on
using ScalarTypes = testing::Types<double>;

template <typename ScalarType_>
class ScalarTest : public testing::Test {
 protected:
    using ScalarType = ScalarType_;
    using Leaf = wave::Scalar<ScalarType>;
    using Translation = wave::Translation<Eigen::Matrix<ScalarType, 3, 1>>;
    using Vector = typename wave::internal::traits<Translation>::ImplType;

    // Static traits checks
    TICK_TRAIT_CHECK(wave::internal::is_scalar<ScalarType>);
    TICK_TRAIT_CHECK(wave::internal::is_leaf_expression<Leaf>);
    TICK_TRAIT_CHECK(wave::internal::is_leaf_expression<wave::Scalar<ScalarType>>);
    TICK_TRAIT_CHECK(wave::internal::is_unary_expression<wave::Scalar<ScalarType &>>);

    // Make some semi-random values, different for each test
    // (too big or small can ruin numerical diff accuracy for some functions, e.g. if
    // dividing by near-zero)
    const ScalarType rand_a =
      wave::uniformRandom<ScalarType>(ScalarType{0.5}, ScalarType{3.0});
    const ScalarType rand_b =
      wave::uniformRandom<ScalarType>(ScalarType{0.5}, ScalarType{3.0});

    // Return them as rvalues
    ScalarType a() const {
        return this->rand_a;
    }
    ScalarType b() const {
        return this->rand_b;
    }
};

#endif  // WAVE_GEOMETRY_SCALAR_EXPRESSION_TEST_HPP
