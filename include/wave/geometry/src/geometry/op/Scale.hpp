/**
 * @file
 */

#ifndef WAVE_GEOMETRY_SCALE_HPP
#define WAVE_GEOMETRY_SCALE_HPP

namespace wave {

/** An expression representing left scalar multiplication of a vector.
 */
template <typename Lhs, typename Rhs>
struct Scale : internal::base_tmpl_t<Rhs, Scale<Lhs, Rhs>>,
               BinaryStorage<Scale<Lhs, Rhs>> {
    // Inherit constructor from BinaryStorage
    using BinaryStorage<Scale<Lhs, Rhs>>::BinaryStorage;
};


/** An expression representing right scalar multiplication of a vector.
 */
template <typename Lhs, typename Rhs>
struct ScaleR : internal::base_tmpl_t<Lhs, ScaleR<Lhs, Rhs>>,
                BinaryStorage<ScaleR<Lhs, Rhs>> {
    // Inherit constructor from BinaryStorage
    using BinaryStorage<ScaleR<Lhs, Rhs>>::BinaryStorage;
};

/** An expression representing the division of a vector (Lhs) by a scalar (Rhs).
 * Equivalent to right scalar multiplication by the inverse of the scalar.
 */
template <typename Lhs, typename Rhs>
struct ScaleDiv : internal::base_tmpl_t<Lhs, ScaleDiv<Lhs, Rhs>>,
                  BinaryStorage<ScaleDiv<Lhs, Rhs>> {
    // Inherit constructor from BinaryStorage
    using BinaryStorage<ScaleDiv<Lhs, Rhs>>::BinaryStorage;
};


namespace internal {

// Traits for left scalar multiplication
// Keep the frames of the vector side.
// @todo add scalar mixing. Currently the type of Rhs is kept.
template <typename Lhs, typename Rhs>
struct traits<Scale<Lhs, Rhs>> : binary_traits_base<Scale<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Rhs>, MiddleFrameOf<Rhs>, RightFrameOf<Rhs>>;
};

// Traits for right scalar multiplication
// Keep the frames of the vector side.
// @todo add scalar mixing. Currently the type of Lhs is kept.
template <typename Lhs, typename Rhs>
struct traits<ScaleR<Lhs, Rhs>> : binary_traits_base<ScaleR<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Lhs>, RightFrameOf<Lhs>>;
};

// Traits for right scalar division
// Keep the frames of the vector side.
// @todo add scalar mixing. Currently the type of Lhs is kept.
template <typename Lhs, typename Rhs>
struct traits<ScaleDiv<Lhs, Rhs>> : binary_traits_base<ScaleDiv<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, MiddleFrameOf<Lhs>, RightFrameOf<Lhs>>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_SCALE_HPP
