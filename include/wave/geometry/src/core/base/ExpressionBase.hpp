/**
 * @file
 */

#ifndef WAVE_GEOMETRY_EXPRESSIONBASE_HPP
#define WAVE_GEOMETRY_EXPRESSIONBASE_HPP

namespace wave {

/** Base class for all expressions */
template <typename Derived>
class ExpressionBase {
 private:
    using OutputType = internal::plain_output_t<Derived>;

    static_assert(!std::is_same<OutputType, internal::NotImplemented>{},
                  "Expression is not evaluable");

 public:
    /** Return reference to this as derived object */
    inline Derived &derived() & noexcept {
        return *static_cast<Derived *>(this);
    }
    /** Return reference to this as derived object */
    inline const Derived &derived() const &noexcept {
        return *static_cast<Derived const *>(this);
    }

    /** Return reference to this as derived object, when this is rvalue */
    inline Derived &&derived() && noexcept {
        return std::move(*static_cast<Derived *>(this));
    }

    /** Evaluate the expression and construct a leaf expression
     *
     * @returns a leaf expression of "plain-old-object" type. (Applies OutputType).
     *
     * @todo remove. Instead, use free function eval(expr) Having .eval() defined here
     * makes it hard to find the correct result type, since Derived is incomplete. The
     * workaround is redundant, error-prone definitions in traits<> classes.
     */
    OutputType eval() const {
        return internal::evaluateTo<OutputType>(this->derived());
    }

    /** Evaluate the jacobian w.r.t. the target */
    template <typename TargetDerived,
              TICK_REQUIRES(internal::is_leaf_expression<TargetDerived>{})>
    auto jacobian(const ExpressionBase<TargetDerived> &wrt) const
      -> internal::jacobian_t<Derived, TargetDerived> {
        return internal::evaluateJacobianAuto(this->derived(), wrt.derived());
    }

    /** Evaluate the value and jacobians w.r.t. all leaves, in reverse mode.
     * This requires that the expression is a tree with unique types.
     */
    auto evalWithJacobians() const -> internal::eval_with_reverse_jacobians_t<Derived> {
        static_assert(internal::unique_leaves_t<Derived>{},
                      "Calling evalWithJacobians() without arguments is only possible "
                      "for expression trees with unique types");
        return internal::evaluateWithReverseJacobians(this->derived());
    }


    /** Evaluate the value and jacobians w.r.t. some targets */
    template <typename... Targets>
    auto evalWithJacobians(const ExpressionBase<Targets> &... wrt) const
      -> std::tuple<OutputType, internal::jacobian_t<Derived, Targets>...> {
        return internal::evaluateWithJacobiansAuto(this->derived(), wrt.derived()...);
    }
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_EXPRESSIONBASE_HPP
