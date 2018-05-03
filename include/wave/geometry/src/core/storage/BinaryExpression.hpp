/**
 * @file
 */

#ifndef WAVE_GEOMETRY_BINARYEXPRESSION_HPP
#define WAVE_GEOMETRY_BINARYEXPRESSION_HPP

namespace wave {

template <template <typename, typename> class Tmpl_,
          typename LhsDerived_,
          typename RhsDerived_>
struct BinaryExpression<Tmpl_<LhsDerived_, RhsDerived_>> {
    using LhsDerived = tmp::remove_cr_t<LhsDerived_>;
    using RhsDerived = tmp::remove_cr_t<RhsDerived_>;
    using LhsDerivedOrig = LhsDerived_;
    using RhsDerivedOrig = RhsDerived_;

    // Hold a reference to leaf expressions to avoid copies, but a copy of other
    // expressions to avoid references to temporaries.
    using LhsStore = internal::wave_ref_sel_t<LhsDerived_>;
    using RhsStore = internal::wave_ref_sel_t<RhsDerived_>;

    using Derived = Tmpl_<LhsDerived, RhsDerived>;

    template <typename LhsArg, typename RhsArg>
    BinaryExpression(LhsArg &&l, RhsArg &&r)
        : lhs_{std::forward<LhsArg>(l)}, rhs_{std::forward<RhsArg>(r)} {}

    BinaryExpression() = delete;
    BinaryExpression(const BinaryExpression &) = default;
    BinaryExpression(BinaryExpression &&) = default;
    BinaryExpression &operator=(const BinaryExpression &) = default;
    BinaryExpression &operator=(BinaryExpression &&) = default;

    const LhsStore &lhs() const & {
        return lhs_;
    }

    const RhsStore &rhs() const & {
        return rhs_;
    }

    const LhsStore &lhs() & {
        return lhs_;
    }

    const RhsStore &rhs() & {
        return rhs_;
    }

    LhsStore &&lhs() && {
        return std::move(lhs_);
    }

    RhsStore &&rhs() && {
        return std::move(rhs_);
    }

 protected:
    LhsStore lhs_;
    RhsStore rhs_;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_BINARYEXPRESSION_HPP
