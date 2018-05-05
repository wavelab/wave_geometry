/**
 * @file
 * @author lkoppel
 * Helper functions for writing tests
 */

#ifndef WAVE_GEOMETRY_TEST_HPP
#define WAVE_GEOMETRY_TEST_HPP

#include <iomanip>
#include <gtest/gtest.h>
#include "wave/geometry/debug.hpp"
#include "wave/geometry/src/util/meta/index_sequence.hpp"

/** Check if Eigen matrix is approximately identity. Use with EXPECT_PRED1 */
inline bool IsIdentity(const Eigen::MatrixXd &m) {
    return m.isIdentity();
}

/** Check if Eigen matrix is approximately zero. Use with EXPECT_PRED1 */
inline bool IsZero(const Eigen::MatrixXd &m) {
    return m.isZero();
}

/** Compare Eigen matrices. Use with EXPECT_PRED2 */
inline bool MatricesApprox(const Eigen::MatrixXd &expected,
                           const Eigen::MatrixXd &actual) {
    return expected.isApprox(actual);
}

/** Compare Eigen matrices with the given tolerance. Use with EXPECT_PRED3 */
inline bool MatricesApproxPrec(const Eigen::MatrixXd &expected,
                               const Eigen::MatrixXd &actual,
                               double prec) {
    return expected.isApprox(actual, prec);
}

/** Compare Eigen quaternions. Use with EXPECT_PRED2 */
inline bool QuaternionsApprox(const Eigen::Quaterniond &expected,
                              const Eigen::Quaterniond &actual) {
    return expected.isApprox(actual);
}

/** Compare Eigen AngleAxes. Use with EXPECT_PRED2 */
inline bool AngleAxesApprox(const Eigen::AngleAxisd &expected,
                            const Eigen::AngleAxisd &actual) {
    return expected.isApprox(actual);
}

// Helpers to allow comparison of scalars, in case that is the output (in checkJacobian)
template <typename P, TICK_REQUIRES(std::is_arithmetic<typename P::first_type>{})>
bool IsApprox(typename P::first_type expected, typename P::second_type actual) {
    testing::internal::FloatingPoint<typename P::first_type> a{expected}, b{actual};
    return a.AlmostEquals(b);
}

template <typename P, TICK_REQUIRES(not std::is_arithmetic<typename P::first_type>{})>
bool IsApprox(const typename P::first_type &expected,
              const typename P::second_type &actual) {
    return expected.isApprox(actual);
}


// Predicate for EXPECT_PRED2
// EXPECT_PRED does not support function templates
// We can only have one template argument to avoid commas in the macro, unless we take
// some workaround like above (todo)
template <typename T>
bool IsApproxPrec(const T &expected, const T &actual, double prec) {
    return expected.isApprox(actual, prec);
}

template <typename Matrix>
bool checkApproxCoeffWise(const Matrix &expected, const Matrix &actual, double prec) {
    for (int i = 0; i < expected.rows(); ++i) {
        for (int j = 0; j < expected.cols(); ++j) {
            using std::abs;
            const auto err = abs(expected(i, j) - actual(i, j));
            EXPECT_NEAR(expected(i, j), actual(i, j), prec) << " (" << i << ", " << j
                                                            << ")";
            if (err >= prec) {
                return false;
            }
        }
    }
    return true;
}

/** Test the result of applying a.isApprox(b), printing the objects if false */
// std::pair is a workaround for passing two parameters with commas
// (need to introduce parens)
#define EXPECT_APPROX(a, b) EXPECT_PRED2(IsApprox<decltype(std::make_pair(a, b))>, a, b)

/** Test the result of applying a.isApprox(b) with the given tolerance */
#define EXPECT_APPROX_PREC(a, b, prec) EXPECT_PRED3(IsApproxPrec<decltype(a)>, a, b, prec)

/** Test the result of applying a.isApprox(b), printing the objects if false */
#define ASSERT_APPROX(a, b) ASSERT_PRED2(IsApprox<decltype(std::make_pair(a, b))>, a, b)

/** Test the result of applying a.isApprox(b) with the given tolerance */
#define ASSERT_APPROX_PREC(a, b, prec) ASSERT_PRED3(IsApproxPrec<decltype(a)>, a, b, prec)

/** Compare a numerical and analytical Jacobian, selecting a loose tolerance for the
 * scalar type. */
template <typename Jacobian>
int checkJacobian(const Jacobian &numerical,
                  const Jacobian &analytical,
                  const std::string &msg) {
    using Scalar = typename Jacobian::Scalar;
    // Empirically-chosen loose precision
    // ULPs are irrelevant when finite difference method introduces so much error
    const auto prec = Scalar{1e-4} + 10 * std::sqrt(Eigen::NumTraits<Scalar>::epsilon());

    // Don't use Eigen's isApprox as it doesn't work too well for Jacobians which have a
    // mix of small and non-small numbers. Instead use an elementwise absolute tolerance.
    EXPECT_PRED3(checkApproxCoeffWise<Jacobian>, numerical, analytical, prec) << msg;
    EXPECT_FALSE(analytical.isZero()) << msg;

    return 0;
}

/** Checks that the value (the first element) of all given tuples matches up,
 * and passes the rest to Jacobian checker (which has a looser tolerance).
 *
 * @note this template's parameters enforce that the value and jacobian types must be the
 * same for all variants.
 *
 * @tparam I the indices of the Jacobians in the tuple, so we can pass them on.
 * See `tmp::index_sequence`.
 */
template <typename Value, typename... Jacobians, int... I>
void checkValueAndJacobians(const std::tuple<Value, Jacobians...> &ref,
                            const std::tuple<Value, Jacobians...> &forward_untyped,
                            const std::tuple<Value, Jacobians...> &forward_typed,
                            const std::tuple<Value, Jacobians...> &reverse,
                            wave::tmp::index_sequence<I...>) {
    // Compare all to the value from .eval(), which is separately checked in unit tests
    const auto &eval_result = std::get<0>(ref);
    EXPECT_APPROX(eval_result, std::get<0>(forward_untyped));
    EXPECT_APPROX(eval_result, std::get<0>(forward_typed));
    EXPECT_APPROX(eval_result, std::get<0>(reverse));

    // Compare each of the Jacobian variants to the numerical results
    int foreach[] = {
      (checkJacobian(std::get<I>(ref),
                     std::get<I>(forward_untyped),
                     "Forward Untyped " + std::to_string(I)),
       checkJacobian(std::get<I>(ref),
                     std::get<I>(forward_typed),
                     "Forward Typed " + std::to_string(I)),
       checkJacobian(
         std::get<I>(ref), std::get<I>(reverse), "Reverse " + std::to_string(I)))...};
    (void) foreach;
}

template <typename Value, typename... Jacobians, int... I>
void checkValueAndJacobiansUntyped(const std::tuple<Value, Jacobians...> &ref,
                                   const std::tuple<Value, Jacobians...> &forward_untyped,
                                   wave::tmp::index_sequence<I...>) {
    // Compare all to the value from .eval(), which is separately checked in unit tests
    const auto &eval_result = std::get<0>(ref);
    EXPECT_APPROX(eval_result, std::get<0>(forward_untyped));

    // Compare each of the Jacobian variants to the numerical results
    int foreach[] = {checkJacobian(std::get<I>(ref),
                                   std::get<I>(forward_untyped),
                                   "Forward Untyped " + std::to_string(I))...};
    (void) foreach;
}

template <typename... T>
void printTupleTypes(const std::string &msg, const std::tuple<T...> &, int n_jacobians) {
    ASSERT_EQ(n_jacobians, sizeof...(T)) << "In " << msg << " results";

    std::cerr << std::setw(15) << std::left << msg << " types are: ";
    bool first = true;
    for (auto &&s : {wave::internal::getTypeString<T>()...}) {
        std::cerr << (first ? "" : ", ") << s;
        first = false;
    }
    std::cerr << std::endl;
}

/** This one is here for the case the tuple types do not match */
template <typename Ref, typename ForU, typename ForT, typename Rev, int... Indices>
void checkValueAndJacobians(const Ref &ref,
                            const ForU &for_u,
                            const ForT &for_t,
                            const Rev &rev,
                            wave::tmp::index_sequence<Indices...>) {
    const auto n_jacobians = sizeof...(Indices) + 1;

    printTupleTypes("reference", ref, n_jacobians);
    printTupleTypes("forward untyped", for_u, n_jacobians);
    printTupleTypes("forward typed", for_t, n_jacobians);
    printTupleTypes("reverse", rev, n_jacobians);

    FAIL();
}

/** Evaluate multiple Jacobians of wave expression in forward and reverse mode and (todo)
 * compare them to numerical.
 *
 * Given a list of input types Wrt... we run the following Jacobian evaluators:
 *
 * 1. Numerical, separately for each Wrt
 * 2. Forward untyped
 * 3. Forward typed
 * 4. Reverse (if the expression tree has unique types)
 *
 * Since Reverse evaluator currently returns Jacobian wrt *all* leaves with no way of
 * disabling any, for the purposes of this test function the arguments Wrt... must be
 * exactly in the order they appear in the expression, so they match with the reverse
 * evaluator. Providing them lets us check that the reverse evaulator is actually
 * producing all the expected outputs.
 *
 * We also check that the value given by the evalWithJacobians() type functions is the
 * same as that given by the regular eval().
 *
 * @tparam UniqueExpr defines whether to use this version or the one for non-unique
 * expression. It is set up this way (as opposed to a differently-named function) for
 * the convenience of the CHECK_JACOBIAN macros, through which these functions are
 * expected to be used.
 */
template <bool UniqueExpr, typename Expr, typename... Wrt>
wave::tmp::enable_if_t<UniqueExpr> checkJacobians(const Expr &expr, const Wrt &... wrt) {
    static_assert(wave::internal::unique_leaves_t<Expr>{},
                  "Expression is not a tree with unique types");

    const auto &value_and_numerical = std::tuple_cat(
      std::make_tuple(expr.eval()), wave::evaluateNumericalJacobians(expr, wrt...));
    // Explicitly call typed and untyped evaluators to make sure both work
    const auto &forward_untyped = wave::internal::evaluateWithJacobians(expr, wrt...);
    const auto &forward_typed = wave::internal::evaluateWithTypedJacobians(expr, wrt...);
    const auto &reverse = expr.evalWithJacobians();

    const auto &jac_indices = wave::tmp::make_index_sequence<sizeof...(Wrt), 1>{};
    checkValueAndJacobians(
      value_and_numerical, forward_untyped, forward_typed, reverse, jac_indices);
}

// Version for non-unique expressions
template <bool UniqueExpr, typename Expr, typename... Wrt>
wave::tmp::enable_if_t<not UniqueExpr> checkJacobians(const Expr &expr,
                                                      const Wrt &... wrt) {
    static_assert(!wave::internal::unique_leaves_t<Expr>{},
                  "Expression is actually a tree with unique types");

    const auto &value_and_numerical = std::tuple_cat(
      std::make_tuple(expr.eval()), wave::evaluateNumericalJacobians(expr, wrt...));
    const auto &forward_untyped = expr.evalWithJacobians(wrt...);

    const auto &jac_indices = wave::tmp::make_index_sequence<sizeof...(Wrt), 1>{};
    checkValueAndJacobiansUntyped(value_and_numerical, forward_untyped, jac_indices);
}

/** Helper to call checkJacobian(expr, wrt) with a trace. Requires `Unique` to be a
 * compile-time boolean.
 *
 * If the expression under test is expected to be a tree with unique types, use
 * Unique==true, otherwise, use unique false. The value of Unique will be asserted against
 * the actual properties of expr.
 */
#define CHECK_JACOBIANS(Unique, expr, ...)                                             \
    do {                                                                               \
        SCOPED_TRACE(::testing::UnitTest::GetInstance()->current_test_info()->name()); \
        checkJacobians<Unique>(expr, __VA_ARGS__);                                     \
    } while (0)

/** Ensure the exception is thrown if and only if the condition is true. */
#define EXPECT_THROW_IF(statement, expected_exception, condition) \
    GTEST_AMBIGUOUS_ELSE_BLOCKER_                                 \
    if (condition)                                                \
        EXPECT_THROW(statement, expected_exception);              \
    else                                                          \
    EXPECT_NO_THROW(statement)

namespace Eigen {

// Let gtest print Eigen Quaternion and AngleAxis values
template <typename Scalar>
inline ::std::ostream &operator<<(::std::ostream &os, const Quaternion<Scalar> &q) {
    IOFormat fmt{StreamPrecision, DontAlignCols, ", ", ", "};
    return os << q.w() << ", " << q.vec().format(fmt) << " (wxyz)";
}

template <typename Scalar>
inline ::std::ostream &operator<<(::std::ostream &os,
                                  const Eigen::Map<Quaternion<Scalar>> &q) {
    return os << Quaternion<Scalar>{q};
}

template <typename Scalar>
inline ::std::ostream &operator<<(::std::ostream &os,
                                  const Eigen::Map<const Quaternion<Scalar>> &q) {
    return os << Quaternion<Scalar>{q};
}


template <typename Scalar>
inline ::std::ostream &operator<<(::std::ostream &os, const AngleAxis<Scalar> &a) {
    IOFormat fmt{StreamPrecision, DontAlignCols, ", ", ", "};
    return os << a.angle() << " rad, " << a.axis().format(fmt);
}

}  // namespace Eigen

namespace wave {

// Let gtest print leaf expressions, using their internal values
template <typename Derived, internal::enable_if_leaf_t<Derived, int> = 0>
std::ostream &operator<<(std::ostream &os, const ExpressionBase<Derived> &expr) {
    return os << expr.derived().value();
}

}  // namespace wave

// Example Frames
struct FrameA;
struct FrameB;
struct FrameC;
struct FrameD;

/** Set of parameters testing unframed expressions in the same templated test cases as
 * framed expressions */
template <typename LeafType>
struct UnframedParams {
    static constexpr bool IsFramed = false;
    // Where we use wave::Frame... directly in the tests, make sure they are actually
    // aliases for unframed
    using FrameA = wave::NoFrame;
    using FrameB = wave::NoFrame;
    using FrameC = wave::NoFrame;

    template <typename T, typename... F>
    using Framed = T;

    using Leaf = LeafType;
    using Scalar = typename wave::internal::traits<LeafType>::Scalar;
};

/** Set of parameters for testing framed expressions */
template <typename LeafType>
struct FramedParams : UnframedParams<LeafType> {
    static constexpr bool IsFramed = true;

    // Example frames. We use aliases to avoid printing out the long qualified names of
    // types defined within the test fixture
    using FrameA = ::FrameA;
    using FrameB = ::FrameB;
    using FrameC = ::FrameC;

    template <typename T, typename... F>
    using Framed = wave::Framed<T, F...>;
};

template <typename... LeafTypes>
using test_types_list =
  testing::Types<UnframedParams<LeafTypes>..., FramedParams<LeafTypes>...>;

#endif  // WAVE_GEOMETRY_TEST_HPP
