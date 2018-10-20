/**
 * @file
 */

#ifndef WAVE_GEOMETRY_DYNAMICREVERSEJACOBIANEVALUATOR_HPP
#define WAVE_GEOMETRY_DYNAMICREVERSEJACOBIANEVALUATOR_HPP

namespace wave {
namespace internal {

/** Evaluates Jacobians of an expression graph in reverse mode.
 *
 * Unlike ReverseJacobianEvaluator, this evaluator does not require unique types or a true
 * tree.
 */
template <typename Derived, typename Adjoint, typename = void>
struct DynamicReverseJacobianEvaluator;

/**
 * Each DynamicReverseJacobianEvaluator::jacobian() method returns a map of leaf addresses
 * to Jacobian matrices
 */
template <typename Scalar>
using DynamicReverseResult = MatrixMap<const void *, Scalar>;

/** getLeaves() functions build up a vector of addresses and tangent sizes.
 * It will be sorted later.
 */
using DynamicLeavesVec = std::vector<std::pair<const void *, int>>;

template <typename Derived, enable_if_leaf_or_scalar_t<Derived, int> = 0>
void getLeaves(adl, DynamicLeavesVec &vec, const ExpressionBase<Derived> &expr) {
    vec.emplace_back(&expr.derived(), traits<Derived>::TangentSize);
}

template <typename Derived, enable_if_unary_t<Derived, int> = 0>
void getLeaves(adl, DynamicLeavesVec &vec, const ExpressionBase<Derived> &expr) {
    return getLeaves(adl{}, vec, expr.derived().rhs());
}

template <typename Derived, enable_if_binary_t<Derived, int> = 0>
auto getLeaves(adl, DynamicLeavesVec &vec, const ExpressionBase<Derived> &expr) -> void {
    getLeaves(adl{}, vec, expr.derived().lhs());
    getLeaves(adl{}, vec, expr.derived().rhs());
}

/** Returns a map of address to leaf tangent size for the given expression */
template <typename Derived>
auto getLeavesMap(const Derived &expr, std::size_t expected_size = 0)
  -> DynamicLeavesVec {
    // Get a vector of leaf addresses, with duplicates
    auto vec = DynamicLeavesVec{};
    vec.reserve(expected_size);
    getLeaves(adl{}, vec, expr);

    // Sort the vector
    std::sort(vec.begin(), vec.end(), [](auto &a, auto &b) { return a.first < b.first; });
    // Don't remove duplicates; the flat_map will do it on insert

    return vec;
}

/** Either initialize a matrix in the map, or add to an existing matrix
 * Insert a new element into the map if it's not there */
template <typename Scalar, typename Adjoint>
void updateJacobianMap(DynamicReverseResult<Scalar> &jac_map,
                       const void *target,
                       const Adjoint &adjoint) {
    jac_map[target] += adjoint;
}

/** Specialization for leaf expression */
template <typename Derived, typename Adjoint>
struct DynamicReverseJacobianEvaluator<Derived,
                                       Adjoint,

                                       enable_if_leaf_or_scalar_t<Derived>> {
    using CleanDerived = tmp::remove_cr_t<Derived>;

    WAVE_STRONG_INLINE DynamicReverseJacobianEvaluator(
      DynamicReverseResult<scalar_t<Derived>> &jac_map,
      const Evaluator<CleanDerived> &evaluator,
      const Adjoint &adjoint)
        : evaluator{evaluator} {
        updateJacobianMap(jac_map, &evaluator.expr, adjoint);
    }

 private:
    const Evaluator<CleanDerived> &evaluator;
};

/** Specialization for unary expression */
template <typename Derived, typename Adjoint>
struct DynamicReverseJacobianEvaluator<Derived,
                                       Adjoint,

                                       enable_if_unary_t<Derived>> {
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
    jac_ref_sel_t<SelfJacobian> self_jac;
    jac_ref_sel_t<Adjoint> adjoint;
    jac_ref_sel_t<RhsAdjoint> rhs_adjoint;

    // Nested jacobian-evaluators
    const DynamicReverseJacobianEvaluator<typename traits<Derived>::RhsDerived,
                                          RhsAdjoint>
      rhs_eval;


 public:
    WAVE_STRONG_INLINE DynamicReverseJacobianEvaluator(
      DynamicReverseResult<scalar_t<Derived>> &jac_map,
      const Evaluator<Derived> &evaluator,
      const Adjoint &adjoint_in)
        : evaluator{evaluator},
          self_jac{jacobianImpl(
            get_expr_tag_t<Derived>{}, this->evaluator(), this->evaluator.rhs_eval())},
          adjoint{adjoint_in},
          rhs_adjoint{adjoint * self_jac},
          rhs_eval{jac_map, evaluator.rhs_eval, rhs_adjoint} {}
};

/** Specialization for a binary expression */
template <typename Derived, typename Adjoint>
struct DynamicReverseJacobianEvaluator<Derived,
                                       Adjoint,

                                       enable_if_binary_t<Derived>> {
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
    jac_ref_sel_t<LhsSelfJacobian> lhs_jac;
    jac_ref_sel_t<RhsSelfJacobian> rhs_jac;
    jac_ref_sel_t<Adjoint> adjoint;
    jac_ref_sel_t<LhsAdjoint> lhs_adjoint;
    jac_ref_sel_t<RhsAdjoint> rhs_adjoint;

    // Nested jacobian-evaluators
    const DynamicReverseJacobianEvaluator<typename traits<Derived>::LhsDerived,
                                          LhsAdjoint>
      lhs_eval;
    const DynamicReverseJacobianEvaluator<typename traits<Derived>::RhsDerived,
                                          RhsAdjoint>
      rhs_eval;

 public:
    WAVE_STRONG_INLINE DynamicReverseJacobianEvaluator(
      DynamicReverseResult<scalar_t<Derived>> &jac_map,
      const Evaluator<Derived> &evaluator,
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
          lhs_eval{jac_map, evaluator.lhs_eval, lhs_adjoint},
          rhs_eval{jac_map, evaluator.rhs_eval, rhs_adjoint} {}
};

/** Helper to construct DynamicReverseJacobianEvaluator */
template <typename Derived, typename Adjoint>
WAVE_STRONG_INLINE void evaluateDynamicReverseJacobiansImpl(
  DynamicReverseResult<scalar_t<Derived>> &jac_map,
  const Evaluator<Derived> &v_eval,
  const Adjoint &init_adjoint) {
    // Make the DynamicReverseJacobianEvaluator tree and simultaneously fill the map
    internal::DynamicReverseJacobianEvaluator<Derived, Adjoint>{
      jac_map, v_eval, init_adjoint};
}

/** Get user-facing matrix for one target in DynamicReverseResult map
 */
template <typename Derived, typename Target>
auto evaluateDynamicReverseJacobiansHelper(
  const DynamicReverseResult<scalar_t<Derived>> &jac_map, const Target &target)
  -> jacobian_t<Derived, Target> {
    auto it = jac_map.find(&target);
    if (it != jac_map.end() && it->second.size() != 0) {
        // Convert dynamic matrix to user-facing fixed-size matrix
        return it->second;
    } else {
        // An empty (size 0x0) dynamic matrix indicates "not found": Jacobian is zero
        return jacobian_t<Derived, Target>::Zero();
    }
}

/** Evaluate all leaf Jacobians in reverse mode using an existing Evaluator tree.
 *
 * @param expected_leaves number of leaves to preallocate (a few bytes) for
 * @return result and map of leaf address to Jacobians as dynamic matrices
 */
template <typename Derived>
WAVE_STRONG_INLINE auto evaluateDynamicReverseJacobians(const Evaluator<Derived> &v_eval,
                                                        std::size_t expected_leaves = 256)
  -> DynamicReverseResult<scalar_t<Derived>> {
    const auto leaves = getLeavesMap(v_eval.expr, expected_leaves);
    auto jac_map = MatrixMap<const void *, scalar_t<Derived>>{
      leaves.begin(), leaves.end(), eval_traits<Derived>::TangentSize};
    jac_map.setZero();

    evaluateDynamicReverseJacobiansImpl(jac_map, v_eval, identity_t<Derived>{});
    return jac_map;
}

/** Evaluate result and all Jacobians in reverse mode
 *
 * @return result and map of leaf address to Jacobians as dynamic matrices
 */
template <typename Derived>
WAVE_STRONG_INLINE auto evaluateWithDynamicReverseJacobians(
  const ExpressionBase<Derived> &expr)
  -> std::pair<plain_output_t<Derived>, DynamicReverseResult<scalar_t<Derived>>> {
    // @todo debug-mode checks

    // Get the correct evaluator
    using OutputType = plain_output_t<Derived>;
    const auto &v_eval = prepareEvaluatorTo<OutputType>(expr.derived());

    // Get result and map of leaf Jacobians
    return {prepareOutput(v_eval), evaluateDynamicReverseJacobians(v_eval)};
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_DYNAMICREVERSEJACOBIANEVALUATOR_HPP
