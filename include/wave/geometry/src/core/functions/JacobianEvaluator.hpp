/**
 * @file
 */

#ifndef WAVE_GEOMETRY_JACOBIANEVALUATOR_HPP
#define WAVE_GEOMETRY_JACOBIANEVALUATOR_HPP

namespace wave {
namespace internal {

/** Evaluates an expression's Jacobian with respect to one target in forward mode
 */
template <typename Derived, typename Target, typename = void>
struct JacobianEvaluator;

/** Gets the target to differentiate w.r.t. for a given type.
 *
 * For most types, this is simply a passthrough. Allows special types like Proxy to
 * differentiate w.r.t. another object.
 *
 * @note the unused leaf tag lets us put the function in the internal namespace and still
 * have ADL
 */
template <typename Derived>
auto getWrtTarget(adl, const ExpressionBase<Derived> &target) -> const Derived & {
    return target.derived();
}

template <typename S, std::enable_if_t<internal::is_scalar<S>{}, int> = 0>
auto getWrtTarget(adl, const S &target) -> const S & {
    return target;
}

/** Specialization for expression of same type as the target */
template <typename Derived>
struct JacobianEvaluator<Derived, Derived> {
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
struct JacobianEvaluator<
  Derived,
  Target,
  std::enable_if_t<is_leaf_or_scalar<Derived>{} && !std::is_same<Derived, Target>{}>> {
    WAVE_STRONG_INLINE JacobianEvaluator(const Evaluator<Derived> &, const Target &) {}

    /** Finds (trivial) jacobian of the leaf expression
     *
     * @returns none ("zero") since types do no match
     */
    WAVE_STRONG_INLINE boost::optional<identity_t<Derived>> jacobian() const {
        return boost::none;
    }
};


/** Specialization for unary expression of different type */
template <typename Derived, typename Target>
struct JacobianEvaluator<
  Derived,
  Target,
  std::enable_if_t<is_unary_expression<Derived>{} && !std::is_same<Derived, Target>{}>> {
    using Jacobian = jacobian_t<Derived, Target>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const JacobianEvaluator<typename traits<Derived>::RhsDerived, Target> rhs_eval;

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
  std::enable_if_t<
    is_binary_expression<Derived>{} &&
    contains_same_type<typename traits<Derived>::LhsDerived, Target>::value &&
    contains_same_type<typename traits<Derived>::RhsDerived, Target>::value &&
    !std::is_same<Derived, Target>{}>> {
    using Jacobian = jacobian_t<Derived, Target>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const JacobianEvaluator<typename traits<Derived>::LhsDerived, Target> lhs_eval;
    const JacobianEvaluator<typename traits<Derived>::RhsDerived, Target> rhs_eval;

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
  std::enable_if_t<
    is_binary_expression<Derived>::value &&
    contains_same_type<typename traits<Derived>::LhsDerived, Target>::value &&
    !contains_same_type<typename traits<Derived>::RhsDerived, Target>::value>> {
    using Jacobian = jacobian_t<Derived, Target>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const JacobianEvaluator<typename traits<Derived>::LhsDerived, Target> lhs_eval;

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
  std::enable_if_t<
    is_binary_expression<Derived>{} &&
    !contains_same_type<typename traits<Derived>::LhsDerived, Target>::value &&
    contains_same_type<typename traits<Derived>::RhsDerived, Target>::value>> {
    using Jacobian = jacobian_t<Derived, Target>;

 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const JacobianEvaluator<typename traits<Derived>::RhsDerived, Target> rhs_eval;

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


/** Specialization for n-ary expression */
template <typename Derived, typename Target>
struct JacobianEvaluator<Derived, Target, enable_if_nary_t<Derived>> {
    using Jacobian = BlockMatrix<Derived, Target>;

    // We will construct child JacobianEvaluators only for the types that may contain
    // our target

    template <typename T>
    using Pred = contains_same_type<T, Target>;
    using ChildExprTuple = tmp::filter_t<Pred, typename traits<Derived>::ElementTuple>;

    template <typename T>
    using make_child = JacobianEvaluator<T, Target>;
    using ChildJacEvalTuple = tmp::apply_each_t<make_child, ChildExprTuple>;

    template <std::size_t I>
    using IndexPred =
      contains_same_type<std::tuple_element_t<I, typename traits<Derived>::ElementTuple>,
                         Target>;
    using IndicesOfChildrenToCall = tmp::filter_index_sequence_t<
      IndexPred,
      std::make_index_sequence<traits<Derived>::CompoundSize>>;


 private:
    // Wrapped Evaluator and nested jacobian-evaluators
    const Evaluator<Derived> &evaluator;
    const ChildJacEvalTuple children;

    template <size_t... Is>
    inline static ChildJacEvalTuple expandToChildJacEvalTuple(
      const Evaluator<Derived> &evaluator,
      const Target &target,
      std::index_sequence<Is...>) {
        return ChildJacEvalTuple{
          JacobianEvaluator<typename traits<Derived>::template ElementType<Is>, Target>{
            std::get<Is>(evaluator.children), target}...};
    }

    template <size_t... Is>
    inline Jacobian combineChildJacobians(std::index_sequence<Is...>) const {
        // Fill with zero first. @todo profile/optimize
        auto jac = Jacobian{};
        jac.setZero();
        int foreach[] = {
          (std::get<Is>(children).jacobian()
             ? (jac.template rowsWrt<Is>() = *std::get<Is>(children).jacobian(), 0)
             : 0)...};
        (void) foreach;
        return jac;
    }


 public:
    /** Constructs JacobianEvaluator and its N children */
    WAVE_STRONG_INLINE JacobianEvaluator(const Evaluator<Derived> &evaluator,
                                         const Target &target)
        : evaluator{evaluator},
          children{
            expandToChildJacEvalTuple(evaluator, target, IndicesOfChildrenToCall{})} {}


    /** @returns jacobian matrix if expr contains target type, or zero matrix otherwise.
     *
     * @todo for now, a compound expression just combines others; its own derivative is
     * identity. Therefore its Jacobian is a vertical concatenation of its children's.
     * @todo n-ary jacobians
     */
    WAVE_STRONG_INLINE boost::optional<Jacobian> jacobian() const {
        constexpr auto N = std::tuple_size<ChildJacEvalTuple>::value;
        if (N > 0) {
            using Indices = std::make_index_sequence<N>;
            return this->combineChildJacobians(Indices{});
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
    const auto j_eval = internal::JacobianEvaluator<Derived, Target>{v_eval, target};
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
    return evaluateOneJacobian(v_eval, getWrtTarget(adl{}, target));
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
                           evaluateOneJacobian(v_eval, getWrtTarget(adl{}, targets))...);
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_JACOBIANEVALUATOR_HPP
