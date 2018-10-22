/**
 * @file
 */

#ifndef WAVE_GEOMETRY_TRANSLATION_HPP
#define WAVE_GEOMETRY_TRANSLATION_HPP

namespace wave {

/** A translation or point in R^3, with its own storage
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Vector3d or
 * Eigen::Map<Eigen::Vector3f>)
 *
 * The alias Translationd is provided for the typical storage type, Eigen::Vector3d.
 */
template <typename ImplType>
class Translation : public TranslationBase<Translation<ImplType>>,
                    public LeafStorage<ImplType, Translation<ImplType>> {
    static_assert(internal::is_eigen_vector<3, ImplType>::value,
                  "ImplType must be an Eigen 3-vector type.");
    using Scalar = typename ImplType::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using Storage = LeafStorage<ImplType, Translation<ImplType>>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    Translation() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(Translation)

    /** Construct from Eigen Matrix object */
    template <typename OtherDerived>
    Translation(const Eigen::MatrixBase<OtherDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Construct from three scalars */
    Translation(Scalar x, Scalar y, Scalar z)
        : Storage{typename Storage::init_storage{}, x, y, z} {}
};

namespace internal {

template <typename ImplType>
struct traits<Translation<ImplType>> : vector_leaf_traits_base<Translation<ImplType>> {};

}  // namespace internal

// Convenience typedefs

using Translationd = Translation<Eigen::Vector3d>;

template <typename F1, typename F2, typename F3>
using TranslationFd = Framed<Translationd, F1, F2, F3>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_TRANSLATION_HPP
