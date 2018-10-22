/**
 * @file
 */

#ifndef WAVE_GEOMETRY_PRODUCT_HPP
#define WAVE_GEOMETRY_PRODUCT_HPP

namespace wave {

/** An expression representing the product of two scalars.
 */
template <typename Lhs, typename Rhs>
struct Product : internal::base_tmpl_t<Lhs, Rhs, Product<Lhs, Rhs>>,
                 internal::binary_storage_for<Product<Lhs, Rhs>> {
    // Inherit constructor from BinaryStorage
    using Storage = internal::binary_storage_for<Product<Lhs, Rhs>>;
    using Storage::Storage;
};


namespace internal {

// Traits for left scalar multiplication
// Keep the frames of the vector side.
// @todo add scalar mixing. Currently the type of Lhs is kept.
template <typename Lhs, typename Rhs>
struct traits<Product<Lhs, Rhs>> : binary_traits_base<Product<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Lhs>, RightFrameOf<Lhs>>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_PRODUCT_HPP
