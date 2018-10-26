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

template <typename Derived>
struct Evaluator<Derived &&> {
    static_assert(tmp::alwaysFalse<Derived>(),
                  "Internal error: Evaluator must be instantiated with clean type");
};

/** Specialization for leaf expression */
template <typename Derived>
struct Evaluator<Derived, enable_if_leaf_t<Derived>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EvalType = eval_t<Derived>;

    WAVE_STRONG_INLINE explicit Evaluator(const Derived &expr)
        : expr{expr}, result{evalImpl(get_expr_tag_t<Derived>(), expr)} {}

    const EvalType &operator()() const {
        return this->result;
    }

 public:
    const eval_storage_t<Derived> expr;
    const EvalType result;
};

/** Specialization for scalar type */
template <typename Derived>
struct Evaluator<Derived, enable_if_scalar_t<Derived>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EvalType = Derived;
    WAVE_STRONG_INLINE explicit Evaluator(const Derived &scalar) : expr{scalar} {}
    const EvalType &operator()() const {
        return this->expr;
    }

 public:
    const eval_storage_t<Derived> expr;
};

/** Specialization for unary expression */
template <typename Derived>
struct Evaluator<Derived, enable_if_unary_t<Derived>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EvalType = eval_t<Derived>;
    using RhsEval = Evaluator<typename traits<Derived>::RhsDerived>;

    WAVE_STRONG_INLINE explicit Evaluator(const Derived &expr)
        : expr{expr},
          rhs_eval{expr.rhs()},
          result{evalImpl(get_expr_tag_t<Derived>(), this->rhs_eval())} {}

    const EvalType &operator()() const {
        return this->result;
    }

 public:
    const eval_storage_t<Derived> expr;
    const RhsEval rhs_eval;
    const EvalType result;
};

/** Specialization for a binary expression */
template <typename Derived>
struct Evaluator<Derived, enable_if_binary_t<Derived>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EvalType = eval_t<Derived>;
    using LhsEval = Evaluator<typename traits<Derived>::LhsDerived>;
    using RhsEval = Evaluator<typename traits<Derived>::RhsDerived>;

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
    const eval_storage_t<Derived> expr;
    const LhsEval lhs_eval;
    const RhsEval rhs_eval;
    const EvalType result;
};

/** Specialization for n-ary expression */
template <typename Derived>
struct Evaluator<Derived, enable_if_nary_t<Derived>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EvalType = eval_t<Derived>;
    using ChildEvalTuple =
      tmp::apply_each_t<Evaluator, typename traits<Derived>::ElementTuple>;
    using IndexSeq = std::make_index_sequence<traits<Derived>::CompoundSize>;
    template <size_t... Is>

    inline static ChildEvalTuple expandToChildEvalTuple(const Derived &nary,
                                                        std::index_sequence<Is...>) {
        // Evaluator<...> is needed here to make gcc 5 happy
        return ChildEvalTuple{
          Evaluator<typename traits<Derived>::template ElementType<Is>>{
            nary.template get<Is>()}...};
    }

    template <size_t... Is>
    inline EvalType expandToEvalImpl(std::index_sequence<Is...>) {
        return evalImpl(get_expr_tag_t<Derived>(), std::get<Is>(this->children)()...);
    }

    WAVE_STRONG_INLINE explicit Evaluator(const Derived &expr)
        : expr{expr},
          children{expandToChildEvalTuple(expr, IndexSeq{})},
          result{this->expandToEvalImpl(IndexSeq{})} {}

    const EvalType &operator()() const {
        return this->result;
    }

 public:
    const eval_storage_t<Derived> expr;
    const ChildEvalTuple children;
    const EvalType result;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_EVALUATOR_HPP
