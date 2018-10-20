/**
 * @file
 */

#ifndef WAVE_GEOMETRY_DYNAMICBASE_HPP
#define WAVE_GEOMETRY_DYNAMICBASE_HPP

namespace wave {

/** Base class for expressions with dynamic dispatch
 *
 * @tparam Leaf The leaf type the derived expression will evaluate to
 */
template <typename Leaf>
class DynamicBase {
    TICK_TRAIT_CHECK(internal::is_leaf_expression<Leaf>);
    using Scalar = internal::scalar_t<Leaf>;
    using EvalType = internal::eval_t<Leaf>;
    enum : int { TangentSize = internal::traits<Leaf>::TangentSize };

 public:
    virtual ~DynamicBase() = default;

    /** Appends leaf addresses and tangent sizes to vec
     * @warning should be private (@todo)
     */
    virtual void dynLeaves(internal::DynamicLeavesVec &vec) const = 0;

 private:
    // Virtualized versions of ExpressionBase methods

    /** Returns (non-user-facing) result of evaluation. */
    virtual auto dynEvaluate() const -> EvalType = 0;

    /** Returns Jacobian with respect to object with the given address
     *
     * @param target_ptr address of the target object
     */
    virtual auto dynJacobian(const void *target_ptr) const
      -> Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> = 0;

    /** Returns set of reverse-mode Jacobians with respect to all leaves
     *
     * @param[in,out] jac_map a map of leaf address to dynamic Matrix, to be filled
     * @param init_adjoint the adjoint matrix of the root node
     * @returns a map of leaf address to dynamic Matrix
     */
    virtual void dynReverseDynamic(
      MatrixMap<const void *, Scalar> &jac_map,
      const internal::DynamicMatrix<Scalar> &init_adjoint) const = 0;

    /** Returns set of reverse-mode Jacobians with respect to all leaves
     * @see dynReverseDynamic. Specialization for 1*m adjoints.
     */
    virtual void dynReverse(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::Matrix<Scalar, 1, TangentSize> &init_adjoint) const = 0;

    /** Returns set of reverse-mode Jacobians with respect to all leaves
     * @see dynReverseDynamic. Specialization for 2*m adjoints.
     */
    virtual void dynReverse(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::Matrix<Scalar, 2, TangentSize> &init_adjoint) const = 0;

    /** Returns set of reverse-mode Jacobians with respect to all leaves
     * @see dynReverseDynamic. Specialization for 3*m adjoints.
     */
    virtual void dynReverse(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::Matrix<Scalar, 3, TangentSize> &init_adjoint) const = 0;

    /** Returns set of reverse-mode Jacobians with respect to all leaves
     * @see dynReverseDynamic. Specialization for 6*m adjoints.
     */
    virtual void dynReverse(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::Matrix<Scalar, 6, TangentSize> &init_adjoint) const = 0;

    /** Fallback for matrix sizes for which there is no specialization of dynReverse().
     *
     * Converts the adjoint to dynamic matrix and calls dynReverseDynamic().
     */
    template <int N>
    void dynReverse(MatrixMap<const void *, Scalar> &jac_map,
                    const Eigen::Matrix<Scalar, N, TangentSize> &init_adjoint) const {
        // @todo make dynamic adapter for MatrixMap
        return dynReverseDynamic(jac_map, internal::DynamicMatrix<Scalar>{init_adjoint});
    }

    /** Returns result of evaluation for numerical diff
     *
     * @see EvaluatorWithDelta
     */
    virtual auto dynEvaluateWithDelta(const void *target, int coeff, Scalar delta) const
      -> EvalType = 0;

    // These friend templates (in specializations for Proxy) use the above virtual methods
    template <typename, typename>
    friend struct internal::Evaluator;
    template <typename, typename>
    friend struct internal::EvaluatorWithDelta;
    template <typename, typename, typename>
    friend struct internal::JacobianEvaluator;
    template <typename, typename>
    friend struct internal::DynamicJacobianEvaluator;
    template <typename, typename, typename>
    friend struct internal::DynamicReverseJacobianEvaluator;
    friend class RefProxy<Leaf>;
};  // namespace wave

namespace internal {

// DynamicBase traits are, so far, used only for Jacobian size lookup. This happens when
// the DynamicBase is the target of AD, via getWrtTarget(const Proxy<Leaf> &)
template <typename Leaf>
struct traits<DynamicBase<Leaf>> : traits<Leaf> {};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_DYNAMICBASE_HPP
