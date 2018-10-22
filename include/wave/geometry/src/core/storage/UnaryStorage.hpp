/**
 * @file
 */

#ifndef WAVE_GEOMETRY_UNARYSTORAGE_HPP
#define WAVE_GEOMETRY_UNARYSTORAGE_HPP

namespace wave {

template <typename Derived_, typename RhsDerived_>
struct UnaryStorage {
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
    explicit UnaryStorage(Arg &&arg) : rhs_{std::forward<Arg>(arg)} {}

    // Forward args to rhs (version for 0 or more than 1 argument)
    template <class... Args,
              std::enable_if_t<(sizeof...(Args) != 1) &&
                                 std::is_constructible<RhsStore, Args...>::value,
                               int> = 0>
    explicit UnaryStorage(Args &&... args) : rhs_{std::forward<Args>(args)...} {}

    UnaryStorage() = delete;
    /** Copy constructor */
    UnaryStorage(const UnaryStorage &) = default;
    /** Move constructor */
    UnaryStorage(UnaryStorage &&) = default;
    /** Copy assignment operator */
    UnaryStorage &operator=(const UnaryStorage &) = default;
    /** Move assignment operator */
    UnaryStorage &operator=(UnaryStorage &&) = default;


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
struct UnaryStorageFor<Tmpl<RhsDerived>> : UnaryStorage<Tmpl<RhsDerived>, RhsDerived> {
    using UnaryStorage<Tmpl<RhsDerived>, RhsDerived>::UnaryStorage;
};

// Specialization for unary expression with an extra parameter (such as Convert)
template <template <typename, typename> class Tmpl, typename Aux, typename RhsDerived>
struct UnaryStorageFor<Tmpl<Aux, RhsDerived>>
    : UnaryStorage<Tmpl<Aux, RhsDerived>, RhsDerived> {
    using UnaryStorage<Tmpl<Aux, RhsDerived>, RhsDerived>::UnaryStorage;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_UNARYSTORAGE_HPP
