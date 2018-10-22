/**
 * @file
 */

#ifndef WAVE_GEOMETRY_DIVIDE_HPP
#define WAVE_GEOMETRY_DIVIDE_HPP

namespace wave {

/** An expression representing scalar / scalar division
 */
template <typename Lhs, typename Rhs>
struct Divide : internal::base_tmpl_t<Lhs, Rhs, Divide<Lhs, Rhs>>,
                internal::binary_storage_for<Divide<Lhs, Rhs>> {
    // Inherit constructor from BinaryStorage
    using Storage = internal::binary_storage_for<Divide<Lhs, Rhs>>;
    using Storage::Storage;
};


namespace internal {

// Traits for left scalar multiplication
// Keep the frames of the vector side.
// @todo add scalar mixing. Currently the type of Lhs is kept.
template <typename Lhs, typename Rhs>
struct traits<Divide<Lhs, Rhs>> : binary_traits_base<Divide<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Lhs>, RightFrameOf<Lhs>>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_DIVIDE_HPP
