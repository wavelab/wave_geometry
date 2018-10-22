/**
 * @file
 */

#ifndef WAVE_GEOMETRY_BINARYSTORAGE_HPP
#define WAVE_GEOMETRY_BINARYSTORAGE_HPP

namespace wave {

template <template <typename, typename> class Tmpl_,
          typename LhsDerived_,
          typename RhsDerived_>
struct BinaryStorage<Tmpl_<LhsDerived_, RhsDerived_>> {
    using LhsDerived = tmp::remove_cr_t<LhsDerived_>;
    using RhsDerived = tmp::remove_cr_t<RhsDerived_>;
    using LhsDerivedOrig = LhsDerived_;
    using RhsDerivedOrig = RhsDerived_;

    // Hold a reference to each expression, unless the type is given as T&& -- then
    // store it by value
    using LhsStore = internal::ref_sel_t<LhsDerived_>;
    using RhsStore = internal::ref_sel_t<RhsDerived_>;

    using Derived = Tmpl_<LhsDerived_, RhsDerived_>;

    template <typename LhsArg, typename RhsArg>
    BinaryStorage(LhsArg &&l, RhsArg &&r)
        : lhs_{std::forward<LhsArg>(l)}, rhs_{std::forward<RhsArg>(r)} {}

    BinaryStorage() = delete;
    BinaryStorage(const BinaryStorage &) = default;
    BinaryStorage(BinaryStorage &&) = default;
    BinaryStorage &operator=(const BinaryStorage &) = default;
    BinaryStorage &operator=(BinaryStorage &&) = default;

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

 private:
    LhsStore lhs_;
    RhsStore rhs_;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_BINARYSTORAGE_HPP
