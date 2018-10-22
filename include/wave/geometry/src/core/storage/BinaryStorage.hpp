/**
 * @file
 */

#ifndef WAVE_GEOMETRY_BINARYSTORAGE_HPP
#define WAVE_GEOMETRY_BINARYSTORAGE_HPP

namespace wave {

/** Mixin providing storage and constructors to satisfy the binary expression concept */
template <typename Derived, typename LhsDerived, typename RhsDerived>
struct BinaryStorage {
 private:
    // Hold a reference to each expression, unless the type is given as T&& -- then
    // store it by value
    using LhsStore = internal::ref_sel_t<LhsDerived>;
    using RhsStore = internal::ref_sel_t<RhsDerived>;

 public:
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

namespace internal {
// Helper to get BinaryStorage type for common binary expression templates
template <typename Derived>
struct binary_storage_selector;

template <template <typename, typename> class Tmpl, typename Lhs, typename Rhs>
struct binary_storage_selector<Tmpl<Lhs, Rhs>> {
    using type = BinaryStorage<Tmpl<Lhs, Rhs>, Lhs, Rhs>;
};

// Gets BinaryStorage type for common binary expression templates (saves characters)
template <typename Derived>
using binary_storage_for = typename binary_storage_selector<Derived>::type;

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_BINARYSTORAGE_HPP
