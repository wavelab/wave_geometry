/**
 * @file
 */

#ifndef WAVE_GEOMETRY_DYNAMIC_EXPR_HPP
#define WAVE_GEOMETRY_DYNAMIC_EXPR_HPP

namespace wave {

/** Gives the wrapped expression dynamic dispatch
 *
 * Dynamic can be used with Proxy or RefProxy to build expressions at runtime.
 *
 *
 *
 * @tparam Derived The wrapped expression type
 */
template <typename RhsDerived>
struct Dynamic final : internal::base_tmpl_t<RhsDerived, Dynamic<RhsDerived>>,
                       DynamicBase<internal::plain_output_t<RhsDerived>>,
                       UnaryExpression<Dynamic<RhsDerived>> {
 private:
    using Base = DynamicBase<internal::plain_output_t<RhsDerived>>;
    using CleanType = tmp::remove_cr_t<RhsDerived>;
    using OutputType = internal::plain_output_t<CleanType>;
    using Storage = UnaryExpression<Dynamic<RhsDerived>>;
    using Scalar = internal::scalar_t<CleanType>;
    using MatrixType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using EvalType = internal::eval_t<OutputType>;
    using PreparedType =
      tmp::remove_cr_t<typename internal::traits<RhsDerived>::PreparedType>;
    enum : int { TangentSize = internal::traits<EvalType>::TangentSize };

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Storage::Storage;

 private:
    decltype(auto) constructEvaluator() const {
        auto &&evaluable_expr = internal::PrepareExpr<CleanType>::run(this->rhs());
        using ExprType = tmp::remove_cr_t<decltype(evaluable_expr)>;
        static_assert(std::is_same<ExprType, PreparedType>{}, "Internal sanity check");
        this->lazy_evaluator.emplace(std::move(evaluable_expr));
        return *this->lazy_evaluator;
    }

    decltype(auto) evaluator() const {
        if (!this->lazy_evaluator) {
            this->constructEvaluator();
        }
        return *this->lazy_evaluator;
    }


    void dynLeaves(internal::DynamicLeavesVec &vec) const override {
        getLeaves(internal::adl{}, vec, this->rhs());
    }

    auto dynEvaluate() const -> EvalType override {
        const auto &v_eval = this->constructEvaluator();
        return v_eval();
    }

    auto dynJacobian(const void *target_ptr) const
      -> boost::optional<MatrixType> override {
        // Note we don't reconstruct the evaluator here. This is OK as long as all of our
        // user-facing methods require function evaluation before derivative.
        const auto &v_eval = this->evaluator();
        return internal::evaluateOneDynamicJacobianRaw(v_eval, target_ptr);
    }

    /** Generic implementation of dynReverse*() methods */
    template <typename MatrixDerived>
    inline void dynReverseImpl(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::MatrixBase<MatrixDerived> &init_adjoint) const {
        // Note we don't reconstruct the evaluator here. This is OK as long as all of our
        // user-facing methods require function evaluation before derivative.
        const auto &v_eval = this->evaluator();
        return internal::evaluateDynamicReverseJacobiansImpl(
          jac_map, v_eval, init_adjoint.derived());
    }

    void dynReverseDynamic(
      MatrixMap<const void *, Scalar> &jac_map,
      const internal::DynamicMatrix<Scalar> &init_adjoint) const override {
        return this->dynReverseImpl(jac_map, init_adjoint);
    }

    void dynReverse(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::Matrix<Scalar, 1, TangentSize> &init_adjoint) const override {
        return this->dynReverseImpl(jac_map, init_adjoint);
    }

    void dynReverse(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::Matrix<Scalar, 2, TangentSize> &init_adjoint) const override {
        return this->dynReverseImpl(jac_map, init_adjoint);
    }

    void dynReverse(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::Matrix<Scalar, 3, TangentSize> &init_adjoint) const override {
        return this->dynReverseImpl(jac_map, init_adjoint);
    }

    void dynReverse(
      MatrixMap<const void *, Scalar> &jac_map,
      const Eigen::Matrix<Scalar, 6, TangentSize> &init_adjoint) const override {
        return this->dynReverseImpl(jac_map, init_adjoint);
    }

    auto dynEvaluateWithDelta(const void *target, int coeff, Scalar delta) const
      -> EvalType override {
        return internal::EvaluatorWithDelta<CleanType>{}(
          this->rhs(), target, coeff, delta);
    }

 private:
    mutable boost::optional<internal::Evaluator<PreparedType>> lazy_evaluator;
};

/** Wrap an expression in a Dynamic
 *
 * @see RefProxy for usage example
 */
template <typename Derived>
auto makeDynamic(const ExpressionBase<Derived> &expr) {
    return Dynamic<Derived>{expr.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUE(makeDynamic, Dynamic, ExpressionBase);

/** Return a reference wrapper for a Dynamic object
 *
 * @see RefProxy for usage example
 */
template <typename Leaf>
auto ref(const DynamicBase<Leaf> &dynamic) {
    return RefProxy<Leaf>{dynamic};
}


namespace internal {

template <typename Rhs>
struct traits<Dynamic<Rhs>> : unary_traits_base<Dynamic<Rhs>> {
    using OutputFunctor = typename traits<Rhs>::OutputFunctor;
};

/** Trivial implementation of Dynamic
 * @note This function is only used if the evaluator knows the derived Dynamic type at
 * compile time, and has already evaluated the rhs expression. In this case, the
 * Dynamic
 * does nothing.
 */
template <typename Rhs>
decltype(auto) evalImpl(expr<Dynamic>, const ExpressionBase<Rhs> &rhs) {
    return rhs.derived();
}

/** Trivial implementation of Dynamic derivative
 * @see evalImpl(expr<Dynamic>, ...)
 */
template <typename Val, typename Rhs>
auto jacobianImpl(expr<Dynamic>,
                  const ExpressionBase<Val> &,
                  const ExpressionBase<Rhs> &) {
    return identity_t<Val>{};
}

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_DYNAMIC_EXPR_HPP
