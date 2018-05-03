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
};

/** Scalar expression representing the square of the L2 norm of a vector value
 */
template <typename Rhs>
struct SquaredNorm : ScalarExpression<SquaredNorm<Rhs>>,
                     UnaryExpression<SquaredNorm<Rhs>> {
 private:
    using Storage = UnaryExpression<SquaredNorm<Rhs>>;

 public:
    // Inherit constructors
    using Storage::Storage;
};

/** Scalar expression representing the L2 norm of a vector value
 */
template <typename Rhs>
struct Norm : ScalarExpression<Norm<Rhs>>, UnaryExpression<Norm<Rhs>> {
 private:
    using Storage = UnaryExpression<Norm<Rhs>>;

 public:
    // Inherit constructors
    using Storage::Storage;
};


namespace internal {

template <typename Derived>
struct traits<Random<Derived>> : nullary_traits_base<Random<Derived>> {};

template <typename Rhs>
struct traits<Norm<Rhs>> : unary_traits_base<Norm<Rhs>> {};

template <typename Rhs>
struct traits<SquaredNorm<Rhs>> : unary_traits_base<SquaredNorm<Rhs>> {};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_COMMONEXPRESSIONS_HPP
