/**
 * @file
 */

#ifndef WAVE_GEOMETRY_COMMONEXPRESSIONS_HPP
#define WAVE_GEOMETRY_COMMONEXPRESSIONS_HPP

namespace wave {
/** Nullary expression representing a random instance of a leaf type
 */
template <typename Derived>
struct Random : internal::base_tmpl_t<Derived, Random<Derived>> {
    static_assert(internal::is_leaf_expression<Derived>{},
                  "Random expression can only be made for leaf expressions");

    // Satisfy the Leaf Expression concept
    auto value() const {
        return eval(this->derived()).value();
    }
};

namespace internal {

template <typename Derived>
struct traits<Random<Derived>> : nullary_traits_base<Random<Derived>> {};

}  // namespace internal

/** Scalar expression representing the square of the L2 norm of a vector value
 */
WAVE_SIMPLE_UNARY_EXPRESSION(SquaredNorm, ScalarBase);

/** Scalar expression representing the L2 norm of a vector value
 */
WAVE_SIMPLE_UNARY_EXPRESSION(Norm, ScalarBase);

/** Vector expression representing a vector divided by its L2 norm
 */
WAVE_SPACE_PRESERVING_UNARY_EXPRESSION(Normalize);


/** Scalar expression representing the dot product of two vectors
 */
WAVE_SIMPLE_BINARY_EXPRESSION(Dot, ScalarBase);

/** Scalar expression representing arccos
 */
WAVE_SIMPLE_UNARY_EXPRESSION(ACos, ScalarBase);

}  // namespace wave

#endif  // WAVE_GEOMETRY_COMMONEXPRESSIONS_HPP
