/**
 * @file
 */

#ifndef WAVE_GEOMETRY_TYPEDJACOBIANEVALUATOR_HPP
#define WAVE_GEOMETRY_TYPEDJACOBIANEVALUATOR_HPP

namespace wave {
namespace internal {

/** Functor to evaluate an expression tree with variables identified only by type
 *
 * @warning experimental
 */
template <typename Derived, typename Target, typename = void>
struct TypedJacobianEvaluator;

/** Specialization for expression where Derived is Target */
template <typename Derived>
struct TypedJacobianEvaluator<Derived, Derived> {
    using Target = Derived;
    using Scalar = scalar_t<Derived>;

    WAVE_STRONG_INLINE TypedJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                              const Target &)
        : evaluator{evaluator} {}

    /** Find (trivial) jacobian of the leaf expression
     *
     * @returns identity if types match, zero otherwise
     */

    auto jacobian() const -> identity_t<Derived> {
        return identity_t<Derived>{};
    }

 private:
    const Evaluator<Derived> &evaluator;
};

/** Specialization for unary expression that contains target */
template <typename Derived, typename Target>
struct TypedJacobianEvaluator<
  Derived,
  Target,
  typename std::enable_if<is_unary_expression<Derived>{} &&
                          !std::is_same<Derived, Target>{} &&
                          contains_same_type<Derived, Target>{}>::type> {
 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const TypedJacobianEvaluator<typename Derived::RhsDerived, Target> rhs_eval;

    using SelfJacobian =
      decltype(jacobianImpl(get_expr_tag_t<Derived>{},
                            std::declval<eval_t<Derived>>(),
                            std::declval<eval_t<typename Derived::RhsDerived>>()));
    using RhsJacobian = decltype(rhs_eval.jacobian());
    using Jacobian = decltype(std::declval<SelfJacobian>() * std::declval<RhsJacobian>());

    // Results cache
    jac_ref_sel_t<SelfJacobian> self_jac;
    jac_ref_sel_t<Jacobian> jac;


 public:
    WAVE_STRONG_INLINE TypedJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                              const Target &target)
        : evaluator{evaluator},
          rhs_eval{evaluator.rhs_eval, target},
          self_jac{jacobianImpl(
            get_expr_tag_t<Derived>{}, this->evaluator(), this->evaluator.rhs_eval())},
          jac{self_jac * this->rhs_eval.jacobian()} {}

    /** Calculate the jacobian w.r.t. the given expression
     *
     * @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     */

    const Jacobian &jacobian() const {
        return this->jac;
    }
};

/** Specialization for a binary expression where both sides contain target */
template <typename Derived, typename Target>
struct TypedJacobianEvaluator<
  Derived,
  Target,
  std::enable_if_t<is_binary_expression<Derived>::value &&
                   !std::is_same<Derived, Target>{} &&
                   contains_same_type<typename Derived::LhsDerived, Target>::value &&
                   contains_same_type<typename Derived::RhsDerived, Target>::value>> {
 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const TypedJacobianEvaluator<typename Derived::LhsDerived, Target> lhs_eval;
    const TypedJacobianEvaluator<typename Derived::RhsDerived, Target> rhs_eval;

    using LhsSelfJacobian =
      decltype(leftJacobianImpl(get_expr_tag_t<Derived>{},
                                std::declval<eval_t<Derived>>(),
                                std::declval<eval_t<typename Derived::LhsDerived>>(),
                                std::declval<eval_t<typename Derived::RhsDerived>>()));
    using RhsSelfJacobian =
      decltype(rightJacobianImpl(get_expr_tag_t<Derived>{},
                                 std::declval<eval_t<Derived>>(),
                                 std::declval<eval_t<typename Derived::LhsDerived>>(),
                                 std::declval<eval_t<typename Derived::RhsDerived>>()));
    using LhsJacobian = decltype(lhs_eval.jacobian());
    using RhsJacobian = decltype(rhs_eval.jacobian());
    using Jacobian =
      decltype(std::declval<LhsSelfJacobian>() * std::declval<LhsJacobian>() +
               std::declval<RhsSelfJacobian>() * std::declval<RhsJacobian>());

    // Results cache
    jac_ref_sel_t<LhsSelfJacobian> lhs_jac;
    jac_ref_sel_t<RhsSelfJacobian> rhs_jac;
    jac_ref_sel_t<Jacobian> jac;

 public:
    WAVE_STRONG_INLINE TypedJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                              const Target &target)
        : evaluator{evaluator},
          lhs_eval{evaluator.lhs_eval, target},
          rhs_eval{evaluator.rhs_eval, target},
          lhs_jac{leftJacobianImpl(get_expr_tag_t<Derived>{},
                                   this->evaluator(),
                                   this->evaluator.lhs_eval(),
                                   this->evaluator.rhs_eval())},
          rhs_jac{rightJacobianImpl(get_expr_tag_t<Derived>{},
                                    this->evaluator(),
                                    this->evaluator.lhs_eval(),
                                    this->evaluator.rhs_eval())},
          jac{lhs_jac * this->lhs_eval.jacobian() + rhs_jac * this->rhs_eval.jacobian()} {
    }


    /** Calculate the jacobian w.r.t. the given expression
     *
     * @returns jacobian expression if expr contains target type, or zero matrix
     * otherwise.
     */

    const Jacobian &jacobian() const {
        return this->jac;
    }
};

/** Specialization for a binary expression where only lhs contains target */
template <typename Derived, typename Target>
struct TypedJacobianEvaluator<
  Derived,
  Target,
  std::enable_if_t<is_binary_expression<Derived>::value &&
                   !std::is_same<Derived, Target>{} &&
                   contains_same_type<typename Derived::LhsDerived, Target>::value &&
                   !contains_same_type<typename Derived::RhsDerived, Target>::value>> {
 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const TypedJacobianEvaluator<typename Derived::LhsDerived, Target> lhs_eval;

    using LhsSelfJacobian =
      decltype(leftJacobianImpl(get_expr_tag_t<Derived>{},
                                std::declval<eval_t<Derived>>(),
                                std::declval<eval_t<typename Derived::LhsDerived>>(),
                                std::declval<eval_t<typename Derived::RhsDerived>>()));
    using LhsJacobian = decltype(lhs_eval.jacobian());
    using Jacobian =
      decltype(std::declval<LhsSelfJacobian>() * std::declval<LhsJacobian>());

    // Results cache
    jac_ref_sel_t<LhsSelfJacobian> lhs_jac;
    jac_ref_sel_t<Jacobian> jac;

 public:
    WAVE_STRONG_INLINE TypedJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                              const Target &target)
        : evaluator{evaluator},
          lhs_eval{evaluator.lhs_eval, target},
          lhs_jac{leftJacobianImpl(get_expr_tag_t<Derived>{},
                                   this->evaluator(),
                                   this->evaluator.lhs_eval(),
                                   this->evaluator.rhs_eval())},
          jac{lhs_jac * this->lhs_eval.jacobian()} {}


    /** Calculate the jacobian w.r.t. the given expression
     *
     * @returns jacobian expression if expr contains target type, or zero matrix
     * otherwise.
     */
    const Jacobian &jacobian() const {
        return this->jac;
    }
};

/** Specialization for a binary expression where only rhs contains target */
template <typename Derived, typename Target>
struct TypedJacobianEvaluator<
  Derived,
  Target,
  std::enable_if_t<is_binary_expression<Derived>::value &&
                   !std::is_same<Derived, Target>{} &&
                   !contains_same_type<typename Derived::LhsDerived, Target>::value &&
                   contains_same_type<typename Derived::RhsDerived, Target>::value>> {
 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const TypedJacobianEvaluator<typename Derived::RhsDerived, Target> rhs_eval;

    using RhsSelfJacobian =
      decltype(rightJacobianImpl(get_expr_tag_t<Derived>{},
                                 std::declval<eval_t<Derived>>(),
                                 std::declval<eval_t<typename Derived::LhsDerived>>(),
                                 std::declval<eval_t<typename Derived::RhsDerived>>()));
    using RhsJacobian = decltype(rhs_eval.jacobian());
    using Jacobian =
      decltype(std::declval<RhsSelfJacobian>() * std::declval<RhsJacobian>());

    // Results cache
    jac_ref_sel_t<RhsSelfJacobian> rhs_jac;
    jac_ref_sel_t<Jacobian> jac;

 public:
    WAVE_STRONG_INLINE TypedJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                              const Target &target)
        : evaluator{evaluator},
          rhs_eval{evaluator.rhs_eval, target},
          rhs_jac{rightJacobianImpl(get_expr_tag_t<Derived>{},
                                    this->evaluator(),
                                    this->evaluator.lhs_eval(),
                                    this->evaluator.rhs_eval())},
          jac{rhs_jac * this->rhs_eval.jacobian()} {}


    /** Calculate the jacobian w.r.t. the given expression
     *
     * @returns jacobian expression if expr contains target type, or zero matrix
     * otherwise.
     */

    const Jacobian &jacobian() const {
        return this->jac;
    }
};


/** Evaluate a jacobian of a expression tree by folding it with TypedJacobianEvaluator
 *
 * @note this also calculates the value and discards it
 */
template <typename Derived,
          typename TargetDerived,
          std::enable_if_t<contains_same_type<Derived, TargetDerived>::value, int> = 0>
auto evaluateTypedJacobian(const ExpressionBase<Derived> &expr,
                           const TargetDerived &target)
  -> jacobian_t<Derived, TargetDerived> {
    assert(containsSame(expr, target));

    // Note since we don't return the value, we don't need the user-facing OutputType
    using OutType = eval_output_t<Derived>;
    const auto &v_eval = prepareEvaluatorTo<OutType>(expr.derived());
    internal::TypedJacobianEvaluator<Derived, TargetDerived> j_eval{v_eval, target};
    const auto &result = j_eval.jacobian();
    return result;
}

template <typename Derived,
          typename TargetDerived,
          std::enable_if_t<!contains_same_type<Derived, TargetDerived>::value, int> = 0>
auto evaluateTypedJacobian(const ExpressionBase<Derived> & /*expr*/, const TargetDerived &
                           /*target*/) -> jacobian_t<Derived, TargetDerived> {
    return jacobian_t<Derived, TargetDerived>::Zero();
}

/** Evaluate the result of an expression tree and any number of jacobians
 *
 * @return the value of the expression
 * @param targets N dependent leaf expressions
 */
template <typename Derived, typename... Targets>
WAVE_STRONG_INLINE auto evaluateWithTypedJacobians(const ExpressionBase<Derived> &expr,
                                                   const Targets &... targets)
  -> std::tuple<plain_output_t<Derived>, jacobian_t<Derived, Targets>...> {
    // @todo better checks for validity of arguments
    // @todo debug-mode checks
    // assert(containsSame(expr, target));

    // Get the correct evaluator
    using OutputType = plain_output_t<Derived>;
    const auto &v_eval = prepareEvaluatorTo<OutputType>(expr.derived());
    using ExprType = tmp::remove_cr_t<decltype(v_eval.expr)>;

    return std::forward_as_tuple(
      prepareOutput(v_eval),
      internal::TypedJacobianEvaluator<ExprType, Targets>{v_eval, targets}.jacobian()...);
}

/** Evaluate one Jacobian of an expression, using forward-mode AD.
 *
 * Either TypedJacobianEvaluator or untyped JacobianEvaluator is used, depending on
 * whether the expression is a tree with unique types.
 *
 * @note this also calculates the value and discards it
 */
template <typename Derived,
          typename TargetDerived,
          std::enable_if_t<unique_leaves_t<Derived>{}, int> = 0>
auto evaluateJacobianAuto(const ExpressionBase<Derived> &expr,
                          const TargetDerived &target)
  -> jacobian_t<Derived, TargetDerived> {
    return evaluateTypedJacobian(expr.derived(), target);
}

template <typename Derived,
          typename TargetDerived,
          std::enable_if_t<!unique_leaves_t<Derived>{}, int> = 0>
auto evaluateJacobianAuto(const ExpressionBase<Derived> &expr,
                          const TargetDerived &target)
  -> jacobian_t<Derived, TargetDerived> {
    return evaluateJacobian(expr.derived(), target);
}

/** Evaluate the result of an expression tree and any number of jacobians
 *
 * Either TypedJacobianEvaluator or untyped JacobianEvaluator is used, depending on
 * whether the expression is a tree with unique types.
 */
template <typename Derived,
          typename... Targets,
          std::enable_if_t<unique_leaves_t<Derived>{}, int> = 0>
auto evaluateWithJacobiansAuto(const ExpressionBase<Derived> &expr,
                               const Targets &... targets)
  -> std::tuple<plain_output_t<Derived>, jacobian_t<Derived, Targets>...> {
    return evaluateWithTypedJacobians(expr.derived(), targets...);
}

template <typename Derived,
          typename... Targets,
          std::enable_if_t<!unique_leaves_t<Derived>{}, int> = 0>
auto evaluateWithJacobiansAuto(const ExpressionBase<Derived> &expr,
                               const Targets &... targets)
  -> std::tuple<plain_output_t<Derived>, jacobian_t<Derived, Targets>...> {
    return evaluateWithJacobians(expr.derived(), targets...);
}


}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_TYPEDJACOBIANEVALUATOR_HPP
