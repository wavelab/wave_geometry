/**
 * @file
 */

#ifndef WAVE_GEOMETRY_UNARYSTORAGE_HPP
#define WAVE_GEOMETRY_UNARYSTORAGE_HPP

namespace wave {

/** Mixin providing storage and constructors to satisfy the unary expression concept */
template <typename Derived, typename RhsDerived_>
struct UnaryStorage {
 private:
    // Hold a reference to leaf expressions to avoid copies, but a copy of other
    // expressions to avoid references to temporaries.
    using RhsStore = internal::storage_t<RhsDerived_>;

    // Helper to lazily instantiate is_constructible check in forwarding constructor
    // We use this inside std::conjunction.
    template <typename...>
    struct copy_not_applicable : std::true_type {};

    // Case for exactly one argument: need to check whether copy constructor applies
    template <typename Arg>
    struct copy_not_applicable<Arg>
        : tmp::bool_constant<!std::is_same<std::decay_t<Arg>, Derived>{}> {};

 public:
    // Forward args to rhs (version for one argument)
    // Disable if copy ctor would apply - https://stackoverflow.com/a/39646176
    template <typename Arg,
              std::enable_if_t<tmp::conjunction<copy_not_applicable<Arg>,
                                                std::is_constructible<RhsStore, Arg>>{},
                               bool> = true>
    explicit UnaryStorage(Arg &&arg) : rhs_{std::forward<Arg>(arg)} {}

    // Forward args to rhs (version for 0 or more than 1 argument)
    template <
      typename... Args,
      std::enable_if_t<tmp::conjunction<copy_not_applicable<Args...>,
                                        std::is_constructible<RhsStore, Args...>>{},
                       bool> = true>
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

    RhsStore &rhs() & {
        return rhs_;
    }

    RhsStore rhs() && {
        return rhs_;
    }

 protected:
    RhsStore rhs_;
};


namespace internal {
// Helper to get UnaryStorage type for common unary expression templates
template <typename Derived>
struct unary_storage_selector;

// Specialization for regular unary expression with one template parameter (such as
// Inverse)
template <template <typename> class Tmpl, typename Rhs>
struct unary_storage_selector<Tmpl<Rhs>> {
    using type = UnaryStorage<Tmpl<Rhs>, Rhs>;
};


// Specialization for unary expression with an extra parameter (such as Convert)
template <template <typename, typename> class Tmpl, typename Aux, typename Rhs>
struct unary_storage_selector<Tmpl<Aux, Rhs>> {
    using type = UnaryStorage<Tmpl<Aux, Rhs>, Rhs>;
};


// Gets BinaryStorage type for common binary expression templates (saves characters)
template <typename Derived>
using unary_storage_for = typename unary_storage_selector<Derived>::type;

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_UNARYSTORAGE_HPP
