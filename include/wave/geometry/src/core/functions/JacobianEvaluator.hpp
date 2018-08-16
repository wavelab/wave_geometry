/**
 * @file
 */

#ifndef WAVE_GEOMETRY_JACOBIANEVALUATOR_HPP
#define WAVE_GEOMETRY_JACOBIANEVALUATOR_HPP

#include <boost/optional.hpp>

namespace wave {
namespace internal {

/** Functor to evaluate an expression tree with variables identified only by type
 *
 * @warning experimental
 */
template <typename Derived, typename Target, typename = void>
struct JacobianEvaluator;

/** Specialization for leaf expression of same type */
template <typename Derived>
struct JacobianEvaluator<Derived, Derived, enable_if_leaf_nullary_or_scalar_t<Derived>> {
    using Scalar = scalar_t<Derived>;

    WAVE_STRONG_INLINE JacobianEvaluator(const Evaluator<Derived> &evaluator,
                                         const Derived &target)
        : evaluator{evaluator}, is_same{isSame(evaluator.expr, target)} {}

    /** Find (trivial) jacobian of the leaf expression
     *
     * @returns identity if types match, zero otherwise
     */

    WAVE_STRONG_INLINE boost::optional<identity_t<Derived>> jacobian() const {
        if (is_same) {
            // Jacobian wrt self is always identity (dx/dx = 1)
            return identity_t<Derived>{};
        } else {
            // Jacobian of a leaf expression wrt something else is always zero.
            // Note we can't know if they're somehow related elsewhere; we only consider
            // the expressions given, and assume leaf expressions are the end of the line.
            return boost::none;
        }
    }

 private:
    const Evaluator<Derived> &evaluator;
    // Value cache
    const bool is_same;
};

/** Specialization for leaf expression of different type */
template <typename Derived, typename Target>
struct JacobianEvaluator<Derived, Target, enable_if_leaf_nullary_or_scalar_t<Derived>> {
    WAVE_STRONG_INLINE JacobianEvaluator(const Evaluator<Derived> &, const Target &) {}

    /** Finds (trivial) jacobian of the leaf expression
     *
     * @returns none ("zero") since types do no match
     */
    WAVE_STRONG_INLINE boost::optional<identity_t<Derived>> jacobian() const {
        return boost::none;
    }
};


/** Specialization for unary expression */
template <typename Derived, typename Target>
struct JacobianEvaluator<Derived, Target, enable_if_unary_t<Derived>> {
    using Jacobian = jacobian_t<Derived, Target>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const JacobianEvaluator<typename Derived::RhsDerived, Target> rhs_eval;

 public:
    WAVE_STRONG_INLINE JacobianEvaluator(const Evaluator<Derived> &evaluator,
                                         const Target &target)
        : evaluator{evaluator}, rhs_eval{evaluator.rhs_eval, target} {}

    /** @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     */
    WAVE_STRONG_INLINE boost::optional<Jacobian> jacobian() const {
        const auto &rhs_jac = this->rhs_eval.jacobian();
        if (rhs_jac) {
            return Jacobian{jacobianImpl(get_expr_tag_t<Derived>{},
                                         this->evaluator(),
                                         this->evaluator.rhs_eval()) *
                            (*rhs_jac)};
        } else {
            return boost::none;
        }
    }
};

/** Specialization for binary expression where both sides *might* contain target */
template <typename Derived, typename Target>
struct JacobianEvaluator<
  Derived,
  Target,
  tmp::enable_if_t<is_binary_expression<Derived>{} &&
                   contains_same_type<typename Derived::LhsDerived, Target>::value &&
                   contains_same_type<typename Derived::RhsDerived, Target>::value>> {
    using Jacobian = jacobian_t<Derived, Target>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const JacobianEvaluator<typename Derived::LhsDerived, Target> lhs_eval;
    const JacobianEvaluator<typename Derived::RhsDerived, Target> rhs_eval;

 public:
    WAVE_STRONG_INLINE JacobianEvaluator(const Evaluator<Derived> &evaluator,
                                         const Target &target)
        : evaluator{evaluator},
          lhs_eval{evaluator.lhs_eval, target},
          rhs_eval{evaluator.rhs_eval, target} {}

    /** @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     */
    WAVE_STRONG_INLINE boost::optional<Jacobian> jacobian() const {
        const auto &lhs_jac = this->lhs_eval.jacobian();
        const auto &rhs_jac = this->rhs_eval.jacobian();
        if (lhs_jac && rhs_jac) {
            return Jacobian{leftJacobianImpl(get_expr_tag_t<Derived>{},
                                             this->evaluator(),
                                             this->evaluator.lhs_eval(),
                                             this->evaluator.rhs_eval()) *
                              (*lhs_jac) +
                            rightJacobianImpl(get_expr_tag_t<Derived>{},
                                              this->evaluator(),
                                              this->evaluator.lhs_eval(),
                                              this->evaluator.rhs_eval()) *
                              (*rhs_jac)};
        } else if (lhs_jac) {
            return Jacobian{leftJacobianImpl(get_expr_tag_t<Derived>{},
                                             this->evaluator(),
                                             this->evaluator.lhs_eval(),
                                             this->evaluator.rhs_eval()) *
                            (*lhs_jac)};
        } else if (rhs_jac) {
            return Jacobian{rightJacobianImpl(get_expr_tag_t<Derived>{},
                                              this->evaluator(),
                                              this->evaluator.lhs_eval(),
                                              this->evaluator.rhs_eval()) *
                            (*rhs_jac)};
        } else {
            return boost::none;
        }
    }
};

/** Specialization for binary expression where only lhs *might* contain target */
template <typename Derived, typename Target>
struct JacobianEvaluator<
  Derived,
  Target,
  tmp::enable_if_t<is_binary_expression<Derived>::value &&
                   contains_same_type<typename Derived::LhsDerived, Target>::value &&
                   !contains_same_type<typename Derived::RhsDerived, Target>::value>> {
    using Jacobian = jacobian_t<Derived, Target>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const JacobianEvaluator<typename Derived::LhsDerived, Target> lhs_eval;

 public:
    WAVE_STRONG_INLINE JacobianEvaluator(const Evaluator<Derived> &evaluator,
                                         const Target &target)
        : evaluator{evaluator}, lhs_eval{evaluator.lhs_eval, target} {}

    /** @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     */
    WAVE_STRONG_INLINE boost::optional<Jacobian> jacobian() const {
        const auto &lhs_jac = this->lhs_eval.jacobian();
        if (lhs_jac) {
            return Jacobian{leftJacobianImpl(get_expr_tag_t<Derived>{},
                                             this->evaluator(),
                                             this->evaluator.lhs_eval(),
                                             this->evaluator.rhs_eval()) *
                            (*lhs_jac)};
        } else {
            return boost::none;
        }
    }
};

/** Specialization for binary expression where only rhs *might* contain target */
template <typename Derived, typename Target>
struct JacobianEvaluator<
  Derived,
  Target,
  tmp::enable_if_t<is_binary_expression<Derived>{} &&
                   !contains_same_type<typename Derived::LhsDerived, Target>::value &&
                   contains_same_type<typename Derived::RhsDerived, Target>::value>> {
    using Jacobian = jacobian_t<Derived, Target>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const JacobianEvaluator<typename Derived::RhsDerived, Target> rhs_eval;

 public:
    WAVE_STRONG_INLINE JacobianEvaluator(const Evaluator<Derived> &evaluator,
                                         const Target &target)
        : evaluator{evaluator}, rhs_eval{evaluator.rhs_eval, target} {}

    /** @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     */
    WAVE_STRONG_INLINE boost::optional<Jacobian> jacobian() const {
        const auto &rhs_jac = this->rhs_eval.jacobian();
        if (rhs_jac) {
            return Jacobian{rightJacobianImpl(get_expr_tag_t<Derived>{},
                                              this->evaluator(),
                                              this->evaluator.lhs_eval(),
                                              this->evaluator.rhs_eval()) *
                            (*rhs_jac)};
        } else {
            return boost::none;
        }
    }
};

/** Evaluate a jacobian using an existing Evaluator tree
 */
template <typename Derived, typename Target>
inline auto evaluateOneJacobian(const Evaluator<Derived> &v_eval, const Target &target)
  -> jacobian_t<Derived, Target> {
    internal::JacobianEvaluator<Derived, Target> j_eval{v_eval, target};
    const auto &result = j_eval.jacobian();
    if (result) {
        return *result;
    } else {
        return jacobian_t<Derived, Target>::Zero();
    }
}

/** Evaluate a jacobian of a expression tree by folding it with
 * internal::JacobianEvaluator as the functor
 *
 * @note this also calculates the value and discards it
 */
template <typename Derived, typename Target>
auto evaluateJacobian(const ExpressionBase<Derived> &expr, const Target &target)
  -> jacobian_t<Derived, Target> {
    // Note since we don't return the value, we don't need the user-facing OutputType
    using OutType = eval_output_t<Derived>;
    const auto &v_eval = prepareEvaluatorTo<OutType>(expr.derived());
    return evaluateOneJacobian(v_eval, target);
}

/** Evaluate the result of an expression tree and any number of jacobians
 *
 * @return the value of the expression
 * @param targets N dependent leaf expressions
 * @param jacobians N matrices for the output
 */
template <typename Derived, typename... Targets>
auto evaluateWithJacobians(const ExpressionBase<Derived> &expr,
                           const Targets &... targets)
  -> std::tuple<plain_output_t<Derived>, jacobian_t<Derived, Targets>...> {
    // Get the value once
    const auto &v_eval = prepareEvaluatorTo<plain_output_t<Derived>>(expr.derived());

    return std::make_tuple(prepareOutput(v_eval),
                           evaluateOneJacobian(v_eval, targets)...);
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_JACOBIANEVALUATOR_HPP
