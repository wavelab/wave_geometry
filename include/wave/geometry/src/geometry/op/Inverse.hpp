/**
 * @file
 */

#ifndef WAVE_GEOMETRY_INVERSE_HPP
#define WAVE_GEOMETRY_INVERSE_HPP

namespace wave {

/** Expression representing the inverse of a rotation
 *
 * @tparam Rhs The type of expression being inverted
 */
template <typename Rhs>
struct Inverse : internal::base_tmpl_t<Rhs, Inverse<Rhs>>, UnaryStorageFor<Inverse<Rhs>> {
    // Inherit storage and constructors from UnaryStorage
    using Storage = UnaryStorageFor<Inverse<Rhs>>;
    using Storage::Storage;
};

namespace internal {

template <typename Rhs>
struct traits<Inverse<Rhs>> : unary_traits_base<Inverse<Rhs>> {
    using OutputFunctor = WrapWithFrames<RightFrameOf<Rhs>, LeftFrameOf<Rhs>>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_INVERSE_HPP
