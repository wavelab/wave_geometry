/**
 * @file
 */

#ifndef WAVE_GEOMETRY_PROXY_HPP
#define WAVE_GEOMETRY_PROXY_HPP

namespace wave {

/** Wrapper for a shared_ptr to a DynamicBase expression
 *
 * This template is used to store a smart pointer to a dynamic expression inside another
 * expression.
 * Proxy<RotationMd> stores a shared_ptr<DynamicBase<RotationMd>>, and hides the pointer
 * semantics. Thus, expressions like Inverse<Proxy<RotationMd>> can be evaluated normally.
 *
 * Assigning and copying from another Proxy is shallow: it rebinds the pointer only.
 *
 * This class satisfies none of the (unary, binary, leaf) concepts.
 * Partial specializations for evaluators are provided below.
 *
 * @tparam Leaf The leaf type the proxy expression will evaluate to
 *
 * @warning Wrapping an expression with references to stack objects in a longer-lasting
 * Proxy will lead to dangling references.
 * @todo Disallow making Proxy of an expression with reference storage.
 */
template <typename Leaf>
class Proxy final : public internal::base_tmpl_t<Leaf, Proxy<Leaf>> {
    TICK_TRAIT_CHECK(internal::is_leaf_expression<Leaf>);

    // Convenience typedef for allocating a Dynamic expression in constructor
    template <typename Derived>
    using DynamicType = Dynamic<internal::arg_t<Derived>>;

 public:
    Proxy() = delete;
    Proxy(const Proxy &) noexcept = default;
    Proxy(Proxy &&) noexcept = default;
    Proxy &operator=(const Proxy &) = default;
    Proxy &operator=(Proxy &&) = default;

    /** Construct by making an rvalue expression Dynamic and moving it to the heap */
    template <typename Derived>
    Proxy(ExpressionBase<Derived> &&expr)
        : storage{std::allocate_shared<DynamicType<Derived>>(
            Eigen::aligned_allocator<DynamicType<Derived>>{},
            DynamicType<Derived>{expr.derived()})} {}

    /** Get a copy of the wrapped smart pointer
     */
    auto get() const noexcept -> std::shared_ptr<DynamicBase<Leaf>> {
        return storage;
    }

    /** Dereference the wrapped smart pointer
     */
    auto follow() const noexcept -> const DynamicBase<Leaf> & {
        return *storage;
    }

 private:
    std::shared_ptr<DynamicBase<Leaf>> storage;
};

/** Moves an rvalue expression to the heap, and returns a Proxy to it
 *
 * @warning Dangling references will result if the expression has references to objects on
 * the stack, and the proxy outlasts it.
 */
template <typename Derived>
auto makeProxy(ExpressionBase<Derived> &&expr) {
    return Proxy<internal::plain_output_t<Derived>>{std::move(expr).derived()};
}

namespace internal {

// Trait to specialize the below templates for only Proxy and related types
template <typename T>
struct is_proxy : std::false_type {};

template <typename Leaf>
struct is_proxy<Proxy<Leaf>> : std::true_type {};

template <typename Derived, typename T = void>
using enable_if_proxy_t = typename std::enable_if<is_proxy<Derived>{}, T>::type;

/* Proxy<L> is a special class that gets an instantiation of Evaluator, PrepareExpr, etc.
 * despite not being a leaf, unary, or binary expression. We provide those partial
 * specializations here.
 */
template <typename Derived>
struct PrepareExpr<Derived, std::enable_if_t<is_proxy<Derived>{}>> {
    static auto run(const Derived &proxy) -> const Derived & {
        return proxy;
    }
};

template <typename Derived>
struct Evaluator<Derived, std::enable_if_t<is_proxy<Derived>{}>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EvalType = eval_t<Derived>;

    WAVE_STRONG_INLINE explicit Evaluator(const Derived &proxy)
        : expr{proxy.follow()}, result{expr.dynEvaluate()} {}

    const EvalType &operator()() const {
        return this->result;
    }

 public:
    const DynamicBase<plain_output_t<Derived>> &expr;
    const EvalType result;
};

template <typename Derived>
struct DynamicJacobianEvaluator<Derived, enable_if_proxy_t<Derived>> {
    using Scalar = scalar_t<Derived>;
    using JacobianType = boost::optional<DynamicMatrix<Scalar>>;
    enum : int { TangentSize = eval_traits<Derived>::TangentSize };

    WAVE_STRONG_INLINE explicit DynamicJacobianEvaluator(const Evaluator<Derived> &v_eval,
                                                         const void *target)
        // Dynamically get the Jacobian of the derived expression
        : v_eval{v_eval}, target{target} {}

    WAVE_STRONG_INLINE JacobianType jacobian() const {
        // Check if Jacobian is wrt this Proxy
        if (isSame(this->v_eval.expr, target)) {
            return DynamicMatrix<Scalar>::Identity(TangentSize, TangentSize).eval();
        }
        // Otherwise, dynamically get the Jacobian of the derived expression
        return this->v_eval.expr.dynJacobian(this->target);
    }

 private:
    const Evaluator<Derived> &v_eval;
    const void *target;
};

template <typename Derived, typename Target>
struct JacobianEvaluator<
  Derived,
  Target,
  std::enable_if_t<is_proxy<Derived>{} && !std::is_same<Derived, Target>{}>>
    : DynamicJacobianEvaluator<Derived> {
    WAVE_STRONG_INLINE explicit JacobianEvaluator(const Evaluator<Derived> &v_eval,
                                                  const Target &target)
        : DynamicJacobianEvaluator<Derived>{v_eval, &target} {}
};

template <typename Derived, typename Adjoint>
struct DynamicReverseJacobianEvaluator<Derived, Adjoint, enable_if_proxy_t<Derived>> {
    WAVE_STRONG_INLINE DynamicReverseJacobianEvaluator(
      DynamicReverseResult<scalar_t<Derived>> &jac_map,
      const Evaluator<Derived> &v_eval,
      const Adjoint &adjoint) {
        // Dynamically get the Jacobians of the derived expression
        v_eval.expr.dynReverse(jac_map, adjoint.eval());
    }
};

template <typename Derived>
struct EvaluatorWithDelta<Derived, enable_if_proxy_t<Derived>> {
    using Scalar = scalar_t<Derived>;
    using PlainType = plain_eval_t<Derived>;

    PlainType operator()(const Derived &proxy,
                         const void *target,
                         int coeff,
                         Scalar delta) const {
        // Dynamically get the value
        const auto &forward_value =
          proxy.follow().dynEvaluateWithDelta(target, coeff, delta);
        return evaluateWithDeltaImpl(
          proxy, target, PlainType{forward_value}, coeff, delta);
    }
};

template <typename Leaf>
struct EvaluatorWithDelta<DynamicBase<Leaf>> {
    using Scalar = scalar_t<Leaf>;
    using PlainType = plain_eval_t<Leaf>;

    PlainType operator()(const DynamicBase<Leaf> &dyn,
                         const void *target,
                         int coeff,
                         Scalar delta) const {
        // Dynamically get the value
        const auto &forward_value = dyn.dynEvaluateWithDelta(target, coeff, delta);
        return evaluateWithDeltaImpl(dyn, target, PlainType{forward_value}, coeff, delta);
    }
};


template <typename Leaf>
struct traits<Proxy<Leaf>> {
    using Tag = expr<Proxy>;
    using PreparedType = Proxy<Leaf> &;
    using EvalType = eval_t<Leaf>;
    using OutputFunctor = typename traits<Leaf>::OutputFunctor;
    using PlainType = Leaf;

    // We don't know the contained leaves at compile time
    using UniqueLeaves = std::false_type;
    using ConvertTo = typename traits<Leaf>::ConvertTo;

    // Store by value in expressions using the proxy, because proxies are rebindable
    static constexpr bool StoreByRef = false;
};

template <typename Leaf>
decltype(auto) getWrtTarget(leaf, const ExpressionBase<Proxy<Leaf>> &proxy) {
    return proxy.derived().follow();
}

template <typename Derived, enable_if_proxy_t<Derived, int> = 0>
void getLeaves(adl, DynamicLeavesVec &vec, const ExpressionBase<Derived> &proxy) {
    return proxy.derived().follow().dynLeaves(vec);
}

// Enabled for DynamicBase references only, not Dynamic<Leaf>
template <typename Leaf>
void getLeaves(adl, DynamicLeavesVec &vec, const DynamicBase<Leaf> &dyn) {
    return dyn.dynLeaves(vec);
}

// Provided only to disambiguate
template <typename Leaf>
void getLeaves(adl, DynamicLeavesVec &vec, const Dynamic<Leaf> &expr) {
    return getLeaves(adl{}, vec, expr.derived().rhs());
}

}  // namespace internal

// For Proxies, identity is determined by what they point at
template <typename Derived, internal::enable_if_proxy_t<Derived, int> = 0>
inline constexpr bool isSame(const ExpressionBase<Derived> &a,
                             const ExpressionBase<Derived> &b) noexcept {
    return &a.derived().follow() == &b.derived().follow();
}

template <typename Leaf>
inline constexpr bool isSame(const DynamicBase<Leaf> &a, const void *b) noexcept {
    return static_cast<const void *>(&a) == b;
}

// Provided only to disambiguate
template <typename Leaf>
inline constexpr bool isSame(const Dynamic<Leaf> &a, const void *b) noexcept {
    return static_cast<const void *>(&a) == b;
}
}  // namespace wave

#endif  // WAVE_GEOMETRY_PROXY_HPP
