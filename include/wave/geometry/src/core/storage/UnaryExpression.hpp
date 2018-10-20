/**
 * @file
 */

#ifndef WAVE_GEOMETRY_UNARYEXPRESSION_HPP
#define WAVE_GEOMETRY_UNARYEXPRESSION_HPP

namespace wave {

template <typename Derived_, typename RhsDerived_>
struct UnaryExpressionBase {
    using Derived = Derived_;
    using RhsDerived = tmp::remove_cr_t<RhsDerived_>;

    // Hold a reference to leaf expressions to avoid copies, but a copy of other
    // expressions to avoid references to temporaries.
    using RhsStore = internal::ref_sel_t<RhsDerived_>;

    // Forward args to rhs (version for one argument)
    // Disable if copy ctor would apply - https://stackoverflow.com/a/39646176
    template <class Arg,
              std::enable_if_t<!std::is_same<std::decay_t<Arg>, Derived>{} &&
                                 std::is_constructible<RhsDerived, Arg &&>{},
                               int> = 0>
    explicit UnaryExpressionBase(Arg &&arg) : rhs_{std::forward<Arg>(arg)} {}

    // Forward args to rhs (version for 0 or more than 1 argument)
    template <class... Args,
              std::enable_if_t<(sizeof...(Args) != 1) &&
                                 std::is_constructible<RhsStore, Args...>::value,
                               int> = 0>
    explicit UnaryExpressionBase(Args &&... args) : rhs_{std::forward<Args>(args)...} {}

    UnaryExpressionBase() = delete;
    /** Copy constructor */
    UnaryExpressionBase(const UnaryExpressionBase &) = default;
    /** Move constructor */
    UnaryExpressionBase(UnaryExpressionBase &&) = default;
    /** Copy assignment operator */
    UnaryExpressionBase &operator=(const UnaryExpressionBase &) = default;
    /** Move assignment operator */
    UnaryExpressionBase &operator=(UnaryExpressionBase &&) = default;


    const RhsStore &rhs() const & {
        return rhs_;
    }

    const RhsStore &rhs() & {
        return rhs_;
    }

    RhsStore &&rhs() && {
        return std::move(rhs_);
    }

 protected:
    RhsStore rhs_;
};

// Specialization for regular unary expression with one template parameter (such as
// Inverse)
template <template <typename> class Tmpl, typename RhsDerived>
struct UnaryExpression<Tmpl<RhsDerived>>
  : UnaryExpressionBase<Tmpl<RhsDerived>, RhsDerived> {
    using UnaryExpressionBase<Tmpl<RhsDerived>, RhsDerived>::UnaryExpressionBase;
};

// Specialization for unary expression with an extra parameter (such as Convert)
template <template <typename, typename> class Tmpl, typename Aux, typename RhsDerived>
struct UnaryExpression<Tmpl<Aux, RhsDerived>>
  : UnaryExpressionBase<Tmpl<Aux, RhsDerived>, RhsDerived> {
    using UnaryExpressionBase<Tmpl<Aux, RhsDerived>, RhsDerived>::UnaryExpressionBase;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_UNARYEXPRESSION_HPP
