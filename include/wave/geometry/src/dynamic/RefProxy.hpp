/**
 * @file
 */

#ifndef WAVE_GEOMETRY_REFPROXY_HPP
#define WAVE_GEOMETRY_REFPROXY_HPP

namespace wave {

/** Acts as a rebindable reference to a Dynamic expression.
 *
 * RefProxy can be used if an expression may take on different values at runtime. The
 * proxy is usable in other expressions, and keeps the expression graph differentiable.
 *
 * To use an expression with RefProxy, it must be wrapped in a Dynamic expression.
 * For example, if x, y, and offset are some expressions,
 *
 *     const auto no_offset = makeDynamic(x);
 *     const auto with_offset = makeDynamic(offset * x);
 *     auto p = ref(no_offset)
 *     auto f = y * p;
 *     if (offset_is_needed) {
 *         p = with_offset;  // Changes the eventual result and derivatives of f
 *     }
 *     return f.evalWithJacobians(...);
 *
 * @warning Like most other expressions, RefProxy will be left dangling if its referent's
 * lifetime ends. For example, the following code would be bad:
 *
 *     if (offset_is_needed) {
 *         const auto with_offset = makeDynamic(a + offset);  // Destroyed when scope ends
 *         c = with_offset;  // Danger!
 *     }
 *     result = d * c;  // c points to expired object; undefined behaviour
 *
 * Assigning and copying from another RefProxy is shallow: it rebinds the reference only.
 *
 * @tparam Leaf The leaf type the proxy expression will evaluate to
 * @note This class satisfies none of the (unary, binary, leaf) concepts.
 */
template <typename Leaf>
class RefProxy final : public internal::base_tmpl_t<Leaf, RefProxy<Leaf>> {
    TICK_TRAIT_CHECK(internal::is_leaf_expression<Leaf>);

 public:
    RefProxy() = delete;

    /** Construct with an lvalue reference to an existing Dynamic expression
     * @note RefProxy cannot be constructed from a temporary expression.
     */
    RefProxy(const DynamicBase<Leaf> &expr) noexcept : ptr{&expr} {}

    /** Gets the reference to the external expression */
    auto follow() const noexcept -> const DynamicBase<Leaf> & {
        return *ptr;
    }

    /** Rebinds to hold a reference to another Dynamic expression
     *
     * Future changes to `other` will not affect this.
     */
    RefProxy &operator=(const RefProxy &other) noexcept = default;


    /** Copy constructs from another RefProxy
     *
     * This proxy will have a reference to the same Dynamic expression. Future changes to
     * `other` will not affect this.
     */
    RefProxy(const RefProxy &other) noexcept = default;

 private:
    DynamicBase<Leaf> const *ptr;
};

namespace internal {

// Reuse Proxy's specializations of evaluators
template <typename Leaf>
struct is_proxy<RefProxy<Leaf>> : std::true_type {};

template <typename Leaf>
struct traits<RefProxy<Leaf>> {
    using Tag = expr<RefProxy>;
    using PreparedType = RefProxy<Leaf> &;
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
decltype(auto) getWrtTarget(leaf, const ExpressionBase<RefProxy<Leaf>> &proxy) {
    return proxy.derived().follow();
}

}  // namespace internal

}  // namespace wave

#endif  // WAVE_GEOMETRY_REFPROXY_HPP
