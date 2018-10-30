/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_COMPOUNDBASE_HPP
#define WAVE_GEOMETRY_COMPOUNDBASE_HPP

namespace wave {

/** @todo */
template <typename Derived>
struct CompoundBase : public ExpressionBase<Derived> {
    template <typename T>
    using BaseTmpl = CompoundBase<T>;
};

/** @todo */
template <typename Derived>
struct CompoundVectorBase : public CompoundBase<Derived> {
 private:
    using Scalar = internal::scalar_t<Derived>;
    enum : int { Size = internal::eval_traits<Derived>::Size };

 public:
    static_assert(
      internal::is_compound_vector_expression<internal::eval_t<Derived>>{},
      "Derived type of CompoundVectorBase must be a compound of vector expressions");

    template <typename T>
    using BaseTmpl = CompoundVectorBase<T>;

    /** Fuzzy comparison - see Eigen::DenseBase::isApprox()
     *
     * Performed elementwise for compound types
     */
    template <typename R,
              TICK_REQUIRES(internal::same_base_tmpl_i<Derived, R>{} and
                            Size == internal::eval_traits<R>::Size)>
    bool isApprox(
      const CompoundVectorBase<R> &rhs,
      const Scalar &prec = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        return internal::compoundIsApprox(
          this->derived().eval(), rhs.derived().eval(), prec);
    }
};


/** Applies the manifold minus operator to two compound elements
 */
template <typename L, typename R, TICK_REQUIRES(internal::same_base_tmpl<L, R>{})>
auto operator-(const CompoundBase<L> &lhs, const CompoundBase<R> &rhs) {
    return CompoundBoxMinus<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                          rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator-,
                                   CompoundBoxMinus,
                                   CompoundBase,
                                   CompoundBase)

/** Applies the manifold plus operator to a compound element
 */
template <typename L, typename R>
auto operator+(const CompoundBase<L> &lhs, const CompoundVectorBase<R> &rhs) {
    return CompoundBoxPlus<internal::cr_arg_t<L>, internal::cr_arg_t<R>>{lhs.derived(),
                                                                         rhs.derived()};
}

WAVE_OVERLOAD_FUNCTION_FOR_RVALUES(operator+,
                                   CompoundBoxPlus,
                                   CompoundBase,
                                   CompoundVectorBase);


namespace internal {

template <typename L, typename R, std::size_t... Is>
auto boxMinusImplExpand(const CompoundBase<L> &lhs,
                        const CompoundBase<R> &rhs,
                        std::index_sequence<Is...>) {
    return makeCompoundLike<typename internal::eval_traits<L>::TangentType>(
      eval(lhs.derived().template get<Is>() - rhs.derived().template get<Is>())...);
}

template <typename L, typename R>
auto evalImpl(expr<CompoundBoxMinus>,
              const CompoundBase<L> &lhs,
              const CompoundBase<R> &rhs) {
    using Indices = std::make_index_sequence<traits<L>::CompoundSize>;
    return boxMinusImplExpand(lhs.derived(), rhs.derived(), Indices{});
}

template <typename L, typename R, std::size_t... Is>
auto boxPlusImplExpand(const CompoundBase<L> &lhs,
                       const CompoundVectorBase<R> &rhs,
                       std::index_sequence<Is...>) {
    return makeCompoundLike<L>(
      eval(lhs.derived().template get<Is>() + rhs.derived().template get<Is>())...);
}

template <typename L, typename R>
auto evalImpl(expr<CompoundBoxPlus>,
              const CompoundBase<L> &lhs,
              const CompoundVectorBase<R> &rhs) {
    using Indices = std::make_index_sequence<traits<L>::CompoundSize>;
    return boxPlusImplExpand(lhs.derived(), rhs.derived(), Indices{});
}


}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_COMPOUNDBASE_HPP
