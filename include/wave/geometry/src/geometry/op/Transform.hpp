/**
 * @file
 */

#ifndef WAVE_GEOMETRY_TRANSFORM_HPP
#define WAVE_GEOMETRY_TRANSFORM_HPP

namespace wave {

/** A rigid transform applied to a vector
 *
 * @tparam Lhs a trasnformation in SE(3)
 * @tparam Rhs a translation in R^3
 * */
template <typename Lhs, typename Rhs>
struct Transform : internal::base_tmpl_t<Rhs, Transform<Lhs, Rhs>>,
                   BinaryStorage<Transform<Lhs, Rhs>> {
    // Inherit constructors from BinaryStorage
    using BinaryStorage<Transform<Lhs, Rhs>>::BinaryStorage;

    static_assert(std::is_same<RightFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Mismatching frames");
};

namespace internal {

template <typename Lhs, typename Rhs>
struct traits<Transform<Lhs, Rhs>> : binary_traits_base<Transform<Lhs, Rhs>> {
    // @todo support flipped sum case
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, LeftFrameOf<Lhs>, RightFrameOf<Rhs>>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_TRANSFORM_HPP
