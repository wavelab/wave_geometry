/**
 * @file
 */

#ifndef WAVE_GEOMETRY_PRINTEXPRESSION_HPP
#define WAVE_GEOMETRY_PRINTEXPRESSION_HPP

#include <iostream>
#include <regex>

#include <boost/version.hpp>
#if BOOST_VERSION < 105600
#include <boost/units/detail/utility.hpp>
#else
#include <boost/core/demangle.hpp>
#endif

namespace wave {
namespace internal {

/** Gets demangled name of a type_info object - call as demangleName(typeid(T));
 *
 * Wrapper to handle different versions of Boost
 */
inline std::string demangleName(const std::type_info &ti) {
#if BOOST_VERSION < 105600
    return boost::units::detail::demangle(ti.name());
#else
    return boost::core::demangle(ti.name());
#endif
}

/** Get the unqualified template name of a type name */
inline std::string getTemplateName(const std::string &full_name, bool include_tp) {
    auto regex = std::regex{"(?:[A-Za-z0-9_]*::)*([A-Za-z0-9_]*)<"};

    if (include_tp) {
        regex = std::regex{"(?:[A-Za-z0-9_]*::)*(.*)"};
    }

    std::smatch matches;
    if (std::regex_search(full_name.begin(), full_name.end(), matches, regex)) {
        return matches[1];
    } else {
        // Unexpected
        return full_name;
    }
}

/** Print demangled name of type T with "const" and "&" qualifiers */
template <typename T>
std::string getTypeString() {
    std::stringstream s;
    const auto &name = demangleName(typeid(T));
    s << (std::is_const<T>{} ? "const" : "");
    s << getTemplateName(name, true);
    s << (std::is_lvalue_reference<T>{} ? "&" : "");
    s << (std::is_rvalue_reference<T>{} ? "&&" : "");
    return s.str();
}

/** Get the unqualified typename of an expression for debugging purposes.*/
template <typename Derived>
inline std::string getExpressionTypeName(const ExpressionBase<Derived> &) {
    const auto full_name = demangleName(typeid(Derived));

    // For leaf expression, include the template parameters (typically ImplType)
    const bool include_template_parameters = is_leaf_expression<Derived>();

    // Extract the unqualified template name
    return getTemplateName(full_name, include_template_parameters);
}

/** Get the unqualified typename of a Conversion expression for debugging purposes
 * This variation includes the first template parameter (the To type) */
template <typename To, typename From>
inline std::string getExpressionTypeName(const Conversion<To, From> &) {
    const auto to_name = demangleName(typeid(To));
    return "Conversion to " + getTemplateName(to_name, true);
}

/** Functor to recursively print the expression tree (for debugging) */
struct PrintExpressionFunctor {
    explicit PrintExpressionFunctor(std::ostream &os) : os{os} {}

    template <typename Derived>
    void operator()(const ExpressionBase<Derived> &expr, int depth) const {
        const auto indent_width = static_cast<std::size_t>(2 * depth);

        this->os << std::string(indent_width, ' ') << " - "
                 << getExpressionTypeName(expr.derived()) << std::endl;
    }

 protected:
    std::ostream &os;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_PRINTEXPRESSION_HPP
