/**
 * @file
 */

#ifndef WAVE_GEOMETRY_DYNAMICJACOBIANEVALUATOR_HPP
#define WAVE_GEOMETRY_DYNAMICJACOBIANEVALUATOR_HPP

namespace wave {
namespace internal {

/** Evaluates an expression's Jacobian with respect to one target in forward mode
 * The target is known only by address.
 */
template <typename Derived, typename = void>
struct DynamicJacobianEvaluator;

// Convenience typedef for dynamic-size matrix
template <typename ScalarType>
using DynamicMatrix = Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>;

/** Specialization for leaf expression */
template <typename Derived>
struct DynamicJacobianEvaluator<Derived, enable_if_leaf_or_scalar_t<Derived>> {
    using DynamicJacobian = DynamicMatrix<scalar_t<Derived>>;
    enum : int { TangentSize = eval_traits<Derived>::TangentSize };

 public:
    WAVE_STRONG_INLINE DynamicJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                                const void *target)
        : evaluator{evaluator}, target{target} {}

    /** Finds Jacobian of the leaf expression
     */
    WAVE_STRONG_INLINE DynamicJacobian jacobian() const {
        if (isSame(evaluator.expr, target)) {
            // Jacobian wrt self is always identity (dx/dx = 1)
            return identity_t<Derived>{};
        } else {
            // Jacobian of a leaf expression wrt something else is always zero, here
            // represented by uninitialized (size 0) dynamic matrix
            // Note we can't know if they're somehow related elsewhere; we only consider
            // the expressions given, and assume leaf expressions are the end of the line.
            return DynamicJacobian{};
        }
    }

 private:
    const Evaluator<Derived> &evaluator;
    const void *target;
};

/** Specialization for unary expression */
template <typename Derived>
struct DynamicJacobianEvaluator<Derived, enable_if_unary_t<Derived>> {
    using DynamicJacobian = DynamicMatrix<scalar_t<Derived>>;
    enum : int { TangentSize = eval_traits<Derived>::TangentSize };
    using RhsEval = DynamicJacobianEvaluator<typename traits<Derived>::RhsDerived>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    // We leave rhs_eval empty (and stop the recursion) if we match the target
    const boost::optional<RhsEval> rhs_eval;

    boost::optional<RhsEval> initializeRhsEval(const Evaluator<Derived> &evaluator,
                                               const void *target) const {
        if (isSame(evaluator.expr, target)) {
            return boost::none;
        }
        return RhsEval{evaluator.rhs_eval, target};
    }

 public:
    WAVE_STRONG_INLINE DynamicJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                                const void *target)
        : evaluator{evaluator}, rhs_eval{initializeRhsEval(evaluator, target)} {}

    /** @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     */
    WAVE_STRONG_INLINE DynamicJacobian jacobian() const {
        if (!this->rhs_eval) {
            return DynamicJacobian::Identity(TangentSize, TangentSize)
              .eval();  // We match the target
        }

        const auto &rhs_jac = this->rhs_eval->jacobian();
        if (rhs_jac.size() > 0) {
            return DynamicJacobian{jacobianImpl(get_expr_tag_t<Derived>{},
                                                this->evaluator(),
                                                this->evaluator.rhs_eval()) *
                                   rhs_jac};
        }
        return DynamicJacobian{};
    }
};

/** Specialization for binary expression where both sides *might* contain target */
template <typename Derived>
struct DynamicJacobianEvaluator<Derived, enable_if_binary_t<Derived>> {
    using DynamicJacobian = DynamicMatrix<scalar_t<Derived>>;
    enum : int { TangentSize = eval_traits<Derived>::TangentSize };
    using LhsEval = DynamicJacobianEvaluator<typename traits<Derived>::LhsDerived>;
    using RhsEval = DynamicJacobianEvaluator<typename traits<Derived>::RhsDerived>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    // We leave lhs_eval and rhs_eval empty (and stop recursing) if we match the target
    const boost::optional<LhsEval> lhs_eval;
    const boost::optional<RhsEval> rhs_eval;

    boost::optional<LhsEval> initializeLhsEval(const Evaluator<Derived> &evaluator,
                                               const void *target) const {
        if (isSame(evaluator.expr, target)) {
            return boost::none;
        }
        return LhsEval{evaluator.lhs_eval, target};
    }

    boost::optional<RhsEval> initializeRhsEval(const Evaluator<Derived> &evaluator,
                                               const void *target) const {
        if (isSame(evaluator.expr, target)) {
            return boost::none;
        }
        return RhsEval{evaluator.rhs_eval, target};
    }


 public:
    WAVE_STRONG_INLINE DynamicJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                                const void *target)
        : evaluator{evaluator},
          lhs_eval{initializeLhsEval(evaluator, target)},
          rhs_eval{initializeRhsEval(evaluator, target)} {}

    /** @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     */
    WAVE_STRONG_INLINE DynamicJacobian jacobian() const {
        if (!this->rhs_eval) {
            // We match the target
            return DynamicJacobian::Identity(TangentSize, TangentSize).eval();
        }
        const auto &lhs_jac = this->lhs_eval->jacobian();
        const auto &rhs_jac = this->rhs_eval->jacobian();
        if (lhs_jac.size() > 0 && rhs_jac.size() > 0) {
            return DynamicJacobian{leftJacobianImpl(get_expr_tag_t<Derived>{},
                                                    this->evaluator(),
                                                    this->evaluator.lhs_eval(),
                                                    this->evaluator.rhs_eval()) *
                                     lhs_jac +
                                   rightJacobianImpl(get_expr_tag_t<Derived>{},
                                                     this->evaluator(),
                                                     this->evaluator.lhs_eval(),
                                                     this->evaluator.rhs_eval()) *
                                     rhs_jac};
        } else if (lhs_jac.size() > 0) {
            return DynamicJacobian{leftJacobianImpl(get_expr_tag_t<Derived>{},
                                                    this->evaluator(),
                                                    this->evaluator.lhs_eval(),
                                                    this->evaluator.rhs_eval()) *
                                   lhs_jac};
        } else if (rhs_jac.size() > 0) {
            return DynamicJacobian{rightJacobianImpl(get_expr_tag_t<Derived>{},
                                                     this->evaluator(),
                                                     this->evaluator.lhs_eval(),
                                                     this->evaluator.rhs_eval()) *
                                   rhs_jac};
        }
        return DynamicJacobian{};
    }
};

/** Specialization for n-ary expression */
template <typename Derived>
struct DynamicJacobianEvaluator<Derived, enable_if_nary_t<Derived>> {
    using DynamicJacobian = DynamicMatrix<scalar_t<Derived>>;
    enum : int { TangentSize = eval_traits<Derived>::TangentSize };
    using IndexSeq = std::make_index_sequence<traits<Derived>::CompoundSize>;
    using ChildJacEvalTuple =
      tmp::apply_each_t<::wave::internal::DynamicJacobianEvaluator,
                        typename traits<Derived>::ElementTuple>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const ChildJacEvalTuple children;

    template <size_t... Is>
    inline static ChildJacEvalTuple expandToChildJacEvalTuple(
      const Evaluator<Derived> &evaluator,
      const void *target,
      std::index_sequence<Is...>) {
        return ChildJacEvalTuple{
          DynamicJacobianEvaluator<typename traits<Derived>::template ElementType<Is>>{
            std::get<Is>(evaluator.children), target}...};
    }

    template <size_t... Is>
    inline DynamicJacobian combineChildJacobians(std::index_sequence<Is...>) const {
        auto jac = DynamicJacobian{};

        // Build up the Jacobian block-by-block. The tricky part is we don't know the
        // width until we get a non-empty child Jacobian. Thus we have to keep track of
        // the row number and fill in the missed blocks with zeros later.
        int row_reached = 0;
        tmp::foreach (
          [&jac, &row_reached](const auto &child) {
              const auto &child_jac = child.jacobian();
              if (child_jac.size() > 0) {
                  if (jac.size() == 0) {
                      // Resize the matrix now that we know the width
                      jac.resize(TangentSize, child_jac.cols());
                      // Fill the previously missed blocks with zero
                      jac.topRows(row_reached).setZero();
                      assert(child_jac.rows() == child.TangentSize);
                  }
                  // Add the next Jacobian
                  jac.middleRows(row_reached, child.TangentSize) = child_jac;
              } else if (jac.size() > 0) {
                  // An empty (meaning zero) child jacobian, but we know the width already
                  jac.middleRows(row_reached, child.TangentSize).setZero();
              }
              row_reached += child.TangentSize;
          },
          std::get<Is>(children)...);
        // Return the matrix whether it is empty (representing zero) or not
        return jac;
    }


 public:
    /** Constructs JacobianEvaluator and its N children */
    WAVE_STRONG_INLINE DynamicJacobianEvaluator(const Evaluator<Derived> &evaluator,
                                                const void *target)
        : evaluator{evaluator},
          children{expandToChildJacEvalTuple(evaluator, target, IndexSeq{})} {}


    /** @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     *
     * @todo for now, a compound expression just combines others; its own derivative is
     * identity. Therefore its Jacobian is a vertical concatenation of its children's.
     * @todo n-ary jacobians
     */
    WAVE_STRONG_INLINE DynamicJacobian jacobian() const {
        return this->combineChildJacobians(IndexSeq{});
    }
};

/** Evaluate a jacobian using an existing Evaluator tree. Use target pointer.
 */
template <typename Derived>
inline auto evaluateOneDynamicJacobianRaw(const Evaluator<Derived> &v_eval,
                                          const void *target)
  -> DynamicMatrix<scalar_t<Derived>> {
    internal::DynamicJacobianEvaluator<Derived> j_eval{v_eval, target};
    return j_eval.jacobian();
}

/** Evaluate a jacobian using an existing Evaluator tree
 */
template <typename Derived, typename Target>
inline auto evaluateOneDynamicJacobian(const Evaluator<Derived> &v_eval,
                                       const Target &target)
  -> jacobian_t<Derived, Target> {
    const auto &actual_target = getWrtTarget(adl{}, target);
    internal::DynamicJacobianEvaluator<Derived> j_eval{v_eval, &actual_target};
    const auto &result = j_eval.jacobian();
    if (result.size() > 0) {
        return jacobian_t<Derived, Target>{result};
    } else {
        return jacobian_t<Derived, Target>::Zero();
    }
}

/** Evaluate a jacobian of a expression tree by folding it with
 * internal::DynamicJacobianEvaluator as the functor
 *
 * @note this also calculates the value and discards it
 */
template <typename Derived, typename Target>
auto evaluateDynamicJacobian(const ExpressionBase<Derived> &expr, const Target &target)
  -> jacobian_t<Derived, Target> {
    // Note since we don't return the value, we don't need the user-facing OutputType
    using OutType = eval_output_t<Derived>;
    const auto &v_eval = prepareEvaluatorTo<OutType>(expr.derived());
    return evaluateOneDynamicJacobian(v_eval, target);
}

/** Evaluate the result of an expression tree and any number of jacobians
 *
 * @return the value of the expression
 * @param targets N dependent leaf expressions
 * @param jacobians N matrices for the output
 */
template <typename Derived, typename... Targets>
auto evaluateWithDynamicJacobians(const ExpressionBase<Derived> &expr,
                                  const Targets &... targets)
  -> std::tuple<plain_output_t<Derived>, jacobian_t<Derived, Targets>...> {
    // Get the value once
    const auto &v_eval = prepareEvaluatorTo<plain_output_t<Derived>>(expr.derived());

    return std::make_tuple(prepareOutput(v_eval),
                           evaluateOneDynamicJacobian(v_eval, targets)...);
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_DYNAMICJACOBIANEVALUATOR_HPP
