/**
 * @file
 */

#ifndef WAVE_GEOMETRY_PERTURB_HPP
#define WAVE_GEOMETRY_PERTURB_HPP

namespace wave {

/** Manifold addition to a spherically normalized homogeneous point
 *
 * @f[ \S4 \times \R3 \to \S4 @f]
 * @todo reorganize / combine with BoxPlus
 */
template <typename Lhs, typename Rhs>
struct HomPlus : internal::base_tmpl_t<Lhs, HomPlus<Lhs, Rhs>>,
                 internal::binary_storage_for<HomPlus<Lhs, Rhs>> {
 private:
    using Storage = internal::binary_storage_for<HomPlus<Lhs, Rhs>>;

 public:
    // Inherit constructors from BinaryStorage
    using Storage::Storage;

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Mismatching frames");
};

/** Manifold subtraction of spherically normalized homogeneous points
 *
 * @f[ \S4 \times \S4 \to \R3 @f]
 * @todo reorganize / combine with BoxPlus
 */
template <typename Lhs, typename Rhs>
struct HomMinus : internal::base_tmpl_t<Lhs, HomMinus<Lhs, Rhs>>,
                  internal::binary_storage_for<HomMinus<Lhs, Rhs>> {
 private:
    using Storage = internal::binary_storage_for<HomMinus<Lhs, Rhs>>;

 public:
    // Inherit constructors from BinaryStorage
    using Storage::Storage;

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Mismatching frames");
};


/** A perturbation of an (unnormalized) homogeneous point by a small vector.
 *
 * @f[ \P3 \times \R3 \to \P3 @f]
 *
 * @note this expression is separate from BoxPlus. It is only valid for small
 * perturbations.
 *
 * Defined in T. Barfoot, State Estimation for Robotics, 2017, eq. 7.324.
 */
template <typename Lhs, typename Rhs>
struct PerturbPlus : internal::base_tmpl_t<Lhs, PerturbPlus<Lhs, Rhs>>,
                     internal::binary_storage_for<PerturbPlus<Lhs, Rhs>> {
 private:
    using Storage = internal::binary_storage_for<PerturbPlus<Lhs, Rhs>>;

 public:
    // Inherit constructors from BinaryStorage
    using Storage::Storage;

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Mismatching frames");
};


/** Difference between two very close (unnormalized) homogeneous points.
 *
 * @f[ \P3 \times \P3 \to \R3 @f]
 *
 * @note this expression is separate from BoxMinus. It is only valid for inputs differing
 * by small perturbations, with the same scale factor.
 *
 * Recovers the perturbation applied via PerturbPlus: (p ~+ v) ~- p = v
 */
template <typename Lhs, typename Rhs>
struct PerturbMinus : internal::base_tmpl_t<Lhs, Rhs, PerturbMinus<Lhs, Rhs>>,
                      internal::binary_storage_for<PerturbMinus<Lhs, Rhs>> {
 private:
    using Storage = internal::binary_storage_for<PerturbMinus<Lhs, Rhs>>;

 public:
    // Inherit constructors from BinaryStorage
    using Storage::Storage;

    static_assert(std::is_same<LeftFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
                  "Mismatching frames");
};


namespace internal {

template <typename Lhs, typename Rhs>
struct traits<HomPlus<Lhs, Rhs>> : binary_traits_base<HomPlus<Lhs, Rhs>> {
    using OutputFunctor = WrapWithFrames<LeftFrameOf<Lhs>, RightFrameOf<Rhs>>;
};

template <typename Lhs, typename Rhs>
struct traits<HomMinus<Lhs, Rhs>> : binary_traits_base<HomMinus<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, RightFrameOf<Rhs>, RightFrameOf<Lhs>>;
};

template <typename Lhs, typename Rhs>
struct traits<PerturbPlus<Lhs, Rhs>> : binary_traits_base<PerturbPlus<Lhs, Rhs>> {
    using OutputFunctor = WrapWithFrames<LeftFrameOf<Lhs>, RightFrameOf<Rhs>>;
};

template <typename Lhs, typename Rhs>
struct traits<PerturbMinus<Lhs, Rhs>> : binary_traits_base<PerturbMinus<Lhs, Rhs>> {
    using OutputFunctor =
      WrapWithFrames<LeftFrameOf<Lhs>, RightFrameOf<Rhs>, RightFrameOf<Lhs>>;
};

}  // namespace internal
}  // namespace wave

#endif  // WAVE_GEOMETRY_PERTURB_HPP
