/**
 * @file
 */

#ifndef WAVE_GEOMETRY_REVERSEJACOBIANEVALUATOR_HPP
#define WAVE_GEOMETRY_REVERSEJACOBIANEVALUATOR_HPP

namespace wave {
namespace internal {

/** Functor to evaluate an expression tree with variables identified only by type
 *
 * @warning experimental
 */
template <typename Derived, typename Target, typename = void>
struct ReverseJacobianEvaluator;

/** Helper template to get the evaluated type of an eigen matrix */
template <typename T>
using eigen_plain_t = typename tmp::remove_cr_t<T>::PlainObject;

/** Specialization for leaf expression */
template <typename Derived, typename Adjoint>
struct ReverseJacobianEvaluator<Derived, Adjoint, enable_if_leaf_t<Derived>> {
    WAVE_STRONG_INLINE ReverseJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                                const Adjoint &adjoint)
        : evaluator{evaluator}, adjoint{adjoint} {}


    auto jacobian() const -> std::tuple<const Adjoint &> {
        return std::forward_as_tuple(adjoint);
    }

 private:
    const Evaluator<Derived> &evaluator;
    const ref_sel_t<Adjoint> adjoint;
};

/** Specialization for unary expression, if types match */
template <typename Derived, typename Adjoint>
struct ReverseJacobianEvaluator<Derived, Adjoint, enable_if_unary_t<Derived>> {
 private:
    using SelfJacobian = decltype(
      jacobianImpl(get_expr_tag_t<Derived>{},
                   std::declval<eval_t<Derived>>(),
                   std::declval<eval_t<typename traits<Derived>::RhsDerived>>()));
    using RhsAdjoint = decltype(std::declval<Adjoint>() * std::declval<SelfJacobian>());


 private:
    // Wrapped Evaluator and
    const Evaluator<Derived> &evaluator;

    // Results cache
    ref_sel_t<SelfJacobian> self_jac;
    ref_sel_t<Adjoint> adjoint;
    ref_sel_t<RhsAdjoint> rhs_adjoint;

    // Nested jacobian-evaluators
    const ReverseJacobianEvaluator<typename traits<Derived>::RhsDerived, RhsAdjoint>
      rhs_eval;


 public:
    WAVE_STRONG_INLINE ReverseJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                                const Adjoint &adjoint_in)
        : evaluator{evaluator},
          self_jac{jacobianImpl(
            get_expr_tag_t<Derived>{}, this->evaluator(), this->evaluator.rhs_eval())},
          adjoint{adjoint_in},
          rhs_adjoint{adjoint * self_jac},
          rhs_eval{evaluator.rhs_eval, rhs_adjoint} {}

    using JacobianTuple = decltype(rhs_eval.jacobian());
    auto jacobian() const -> JacobianTuple {
        return this->rhs_eval.jacobian();
    }
};

/** Specialization for a binary expression */
template <typename Derived, typename Adjoint>
struct ReverseJacobianEvaluator<Derived, Adjoint, enable_if_binary_t<Derived>> {
 private:
    using LhsSelfJacobian = decltype(
      leftJacobianImpl(get_expr_tag_t<Derived>{},
                       std::declval<eval_t<Derived>>(),
                       std::declval<eval_t<typename traits<Derived>::LhsDerived>>(),
                       std::declval<eval_t<typename traits<Derived>::RhsDerived>>()));
    using RhsSelfJacobian = decltype(
      rightJacobianImpl(get_expr_tag_t<Derived>{},
                        std::declval<eval_t<Derived>>(),
                        std::declval<eval_t<typename traits<Derived>::LhsDerived>>(),
                        std::declval<eval_t<typename traits<Derived>::RhsDerived>>()));
    using LhsAdjoint =
      decltype(std::declval<Adjoint>() * std::declval<LhsSelfJacobian>());
    using RhsAdjoint =
      decltype(std::declval<Adjoint>() * std::declval<RhsSelfJacobian>());


 private:
    // Wrapped Evaluator
    const Evaluator<Derived> &evaluator;

    // Results cache
    ref_sel_t<LhsSelfJacobian> lhs_jac;
    ref_sel_t<RhsSelfJacobian> rhs_jac;
    ref_sel_t<Adjoint> adjoint;
    ref_sel_t<LhsAdjoint> lhs_adjoint;
    ref_sel_t<RhsAdjoint> rhs_adjoint;

    // Nested jacobian-evaluators
    const ReverseJacobianEvaluator<typename traits<Derived>::LhsDerived, LhsAdjoint>
      lhs_eval;
    const ReverseJacobianEvaluator<typename traits<Derived>::RhsDerived, RhsAdjoint>
      rhs_eval;

 public:
    WAVE_STRONG_INLINE ReverseJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                                const Adjoint &adjoint_in)
        : evaluator{evaluator},
          lhs_jac{leftJacobianImpl(get_expr_tag_t<Derived>{},
                                   this->evaluator(),
                                   this->evaluator.lhs_eval(),
                                   this->evaluator.rhs_eval())},
          rhs_jac{rightJacobianImpl(get_expr_tag_t<Derived>{},
                                    this->evaluator(),
                                    this->evaluator.lhs_eval(),
                                    this->evaluator.rhs_eval())},
          adjoint{adjoint_in},
          lhs_adjoint{adjoint * lhs_jac},
          rhs_adjoint{adjoint * rhs_jac},
          lhs_eval{evaluator.lhs_eval, lhs_adjoint},
          rhs_eval{evaluator.rhs_eval, rhs_adjoint} {}


    using JacobianTuple =
      decltype(std::tuple_cat(lhs_eval.jacobian(), rhs_eval.jacobian()));
    auto jacobian() const -> JacobianTuple {
        return std::tuple_cat(lhs_eval.jacobian(), rhs_eval.jacobian());
    }
};

/** Helper to make a tuple of values from a tuple of references.
 *
 * This is needed because in the case `size == 1`, calling tuple's constructor
 * tuple<Matrix>(tuple<const Matrix&>) will try to construct a Matrix from a tuple
 * (because of the changes to tuple.cnstr in https://cplusplus.github.io/LWG/issue2549,
 * combined with Eigen::Matrix's overly greedy templated constructor).
 */
template <typename Target, typename T>
Target makeTupleFromTuple(T &&t) {
    return Target{std::forward<T>(t)};
};

// case for size == 1
template <typename Target, typename T>
Target makeTupleFromTuple(const std::tuple<T> &t) {
    return Target{std::get<0>(t)};
};

template <typename Target, typename T>
Target makeTupleFromTuple(std::tuple<T> &&t) {
    return Target{std::get<0>(std::move(t))};
};

template <typename Derived, typename = void>
struct eval_with_reverse_jacobians_impl {
    using type = NotAllowed;
};


template <typename Derived>
struct eval_with_reverse_jacobians_impl<Derived,
                                        tmp::enable_if_t<unique_leaves_t<Derived>{}>> {
    using type = tmp::apply_t<
      std::tuple,
      plain_output_t<Derived>,
      tmp::apply_each_t<jacobian_t, Derived, typename unique_leaves_t<Derived>::type>>;
};

/** The expected return type of evaluateWithReverseJacobians.
 *
 * @warning this alias gives NotAllowed if unique_leaves_t<Derived>::value is false.
 *
 * We need this alias so we can use the return type in ExpressionBase<Derived>, while
 * Derived is an incomplete type.
 *
 * @todo It could also be used *inside* ReverseJacobianEvaluator itself, but that was
 * written separately assuming a complete type for Derived. Therefore, for now, we just
 * make sure the two types match.
 */
template <typename Derived>
using eval_with_reverse_jacobians_t =
  typename eval_with_reverse_jacobians_impl<Derived>::type;


/** Evaluate the result of an expression tree and all jacobians
 *
 * @return a tuple of the value of the expression and all jacobians
 */
template <
  typename Derived,
  typename ExprJacobianTuple =
    typename ReverseJacobianEvaluator<Derived, identity_t<Derived>>::JacobianTuple,
  typename JacobianTuple = tmp::apply_each_t<eigen_plain_t, ExprJacobianTuple>>
WAVE_STRONG_INLINE auto evaluateWithReverseJacobiansImpl(const Evaluator<Derived> &v_eval)
  -> tmp::concat_t<std::tuple<plain_output_t<Derived>>, JacobianTuple> {
    // Make the ReverseJacobianEvaluator tree
    internal::ReverseJacobianEvaluator<Derived, identity_t<Derived>> j_eval{
      v_eval, identity_t<Derived>{}};

    return std::tuple_cat(std::forward_as_tuple(prepareOutput(v_eval)),
                          makeTupleFromTuple<JacobianTuple>(j_eval.jacobian()));
}

/** Evaluate the result of an expression tree and all jacobians
 *
 * @return a tuple of the value of the expression and all jacobians
 */
template <typename Derived, TICK_REQUIRES(unique_leaves_t<Derived>{})>
WAVE_STRONG_INLINE auto evaluateWithReverseJacobians(const ExpressionBase<Derived> &expr)
  -> eval_with_reverse_jacobians_t<Derived> {
    // @todo better checks for validity of arguments
    // @todo debug-mode checks
    // assert(containsSame(expr, target));

    // Make the Evaluator tree (forward sweep)
    using OutputType = plain_output_t<Derived>;
    const auto &v_eval = prepareEvaluatorTo<OutputType>(expr.derived());

    // Make the ReverseJacobianEvaluator tree
    return evaluateWithReverseJacobiansImpl(v_eval);

    using RetType = tmp::remove_cr_t<decltype(evaluateWithReverseJacobiansImpl(v_eval))>;
    static_assert(
      std::is_constructible<eval_with_reverse_jacobians_t<Derived>, RetType>{},
      "Internal sanity check: expected reverse evaluator return type");
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_REVERSEJACOBIANEVALUATOR_HPP
