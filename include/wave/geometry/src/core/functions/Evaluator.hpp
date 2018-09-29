/**
 * @file
 */

#ifndef WAVE_GEOMETRY_EVALUATOR_HPP
#define WAVE_GEOMETRY_EVALUATOR_HPP

namespace wave {
namespace internal {

/** Functor to evaluate an expression tree
 *
 * For now, the expression is evaluated as-is
 * In future, optimizations could go here. For example,
 * q1.inverse() * q2.inverse() -> (q2 * q1).inverse()
 *
 */
template <typename Derived, typename = void>
struct Evaluator;

/** Specialization for leaf expression */
template <typename Derived>
struct Evaluator<Derived, enable_if_leaf_t<Derived>> {
    using EvalType = eval_t<Derived>;

    WAVE_STRONG_INLINE explicit Evaluator(const Derived &expr)
        : expr{expr}, result{evalImpl(get_expr_tag_t<Derived>(), expr)} {}

    const EvalType &operator()() const {
        return this->result;
    }

 public:
    const wave_ref_sel_t<Derived> expr;
    const EvalType result;
};

/** Specialization for scalar type */
template <typename Derived>
struct Evaluator<Derived, std::enable_if_t<is_scalar<Derived>{}>> {
    using EvalType = Derived;
    WAVE_STRONG_INLINE explicit Evaluator(const Derived &scalar) : expr{scalar} {}
    const EvalType &operator()() const {
        return this->expr;
    }

 public:
    const wave_ref_sel_t<Derived> expr;
};

/** Specialization for unary expression */
template <typename Derived>
struct Evaluator<Derived, enable_if_unary_t<Derived>> {
    using EvalType = eval_t<Derived>;
    using RhsEval = Evaluator<typename Derived::RhsDerived>;

    WAVE_STRONG_INLINE explicit Evaluator(const Derived &expr)
        : expr{expr},
          rhs_eval{expr.rhs()},
          result{evalImpl(get_expr_tag_t<Derived>(), this->rhs_eval())} {}

    const EvalType &operator()() const {
        return this->result;
    }

 public:
    const wave_ref_sel_t<Derived> expr;
    const RhsEval rhs_eval;
    const EvalType result;
};

/** Specialization for a binary expression */
template <typename Derived>
struct Evaluator<Derived, enable_if_binary_t<Derived>> {
    using EvalType = eval_t<Derived>;
    using LhsEval = Evaluator<typename Derived::LhsDerived>;
    using RhsEval = Evaluator<typename Derived::RhsDerived>;

    WAVE_STRONG_INLINE explicit Evaluator(const Derived &expr)
        : expr{expr},
          lhs_eval{expr.lhs()},
          rhs_eval{expr.rhs()},
          result{
            evalImpl(get_expr_tag_t<Derived>(), this->lhs_eval(), this->rhs_eval())} {}

    const EvalType &operator()() const {
        return this->result;
    }

 public:
    const wave_ref_sel_t<Derived> expr;
    const LhsEval lhs_eval;
    const RhsEval rhs_eval;
    const EvalType result;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_EVALUATOR_HPP
