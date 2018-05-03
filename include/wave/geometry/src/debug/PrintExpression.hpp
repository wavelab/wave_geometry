/**
 * @file
 */

#ifndef WAVE_GEOMETRY_PRINTEXPRESSION_HPP
#define WAVE_GEOMETRY_PRINTEXPRESSION_HPP

#include <iostream>
#include <regex>
#include <boost/core/demangle.hpp>

namespace wave {
namespace internal {

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

/** Get the unqualified typename of an expression for debugging purposes.*/
template <typename Derived>
inline std::string getExpressionTypeName(const ExpressionBase<Derived> &) {
    const auto full_name = boost::core::demangle(typeid(Derived).name());

    // For leaf expression, include the template parameters (typically ImplType)
    const bool include_template_parameters = is_leaf_expression<Derived>();

    // Extract the unqualified template name
    return getTemplateName(full_name, include_template_parameters);
}

/** Get the unqualified typename of a Conversion expression for debugging purposes
 * This variation includes the first template parameter (the To type) */
template <typename To, typename From>
inline std::string getExpressionTypeName(const Conversion<To, From> &) {
    const auto to_name = boost::core::demangle(typeid(To).name());
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

/** Recursively pretty-print the expression tree.
 *
 * @warning This function relies on boost::core::demangle and
 * std::type_info::name(),
 * which is implementation-defined. It may not work on all platforms, and is
 * intended for
 * debugging use only.
 */
template <typename Tree>
inline void printExpression(const Tree &expr, std::ostream &os = std::cout) {
    visitTree(expr, internal::PrintExpressionFunctor{os});
}

}  // namespace wave

#endif  // WAVE_GEOMETRY_PRINTEXPRESSION_HPP
