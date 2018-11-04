/**
 * @file
 */

#ifndef WAVE_GEOMETRY_HOMOGENEOUSPOINT_HPP
#define WAVE_GEOMETRY_HOMOGENEOUSPOINT_HPP

namespace wave {

/** A point in P^3, with its own storage
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Vector4d or
 * Eigen::Map<Eigen::Vector4f>)
 *
 * The alias HomogeneousPointd is provided for the typical storage type, Eigen::Vector4d.
 */
template <typename ImplType>
class HomogeneousPoint : public HomogeneousPointBase<HomogeneousPoint<ImplType>>,
                         public LeafStorage<HomogeneousPoint<ImplType>, ImplType> {
    static_assert(internal::is_eigen_vector<4, ImplType>::value,
                  "ImplType must be an Eigen 4-vector type.");
    using Scalar = typename ImplType::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using Storage = LeafStorage<HomogeneousPoint<ImplType>, ImplType>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    HomogeneousPoint() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(HomogeneousPoint)

    /** Construct from Eigen Matrix object */
    template <typename OtherDerived>
    HomogeneousPoint(const Eigen::MatrixBase<OtherDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Construct from four scalars */
    HomogeneousPoint(Scalar x, Scalar y, Scalar z, Scalar w) {
        this->storage << x, y, z, w;
    }
};

namespace internal {

template <typename ImplType>
struct traits<HomogeneousPoint<ImplType>>
    : point_leaf_traits_base<HomogeneousPoint<ImplType>> {
 private:
    using EigenVector3 = Eigen::Matrix<typename ImplType::Scalar, 3, 1>;

 public:
    using TangentType = Translation<EigenVector3>;
    using TangentBlocks = tmp::type_list<TangentType>;
};

}  // namespace internal

// Convenience typedefs

using HomogeneousPointd = HomogeneousPoint<Eigen::Vector4d>;

/** A 3D point whose position is defined by by the vector TranslationFd<F1, F1, F2>.
 */
template <typename F1, typename F2>
using HomogeneousPointFd = Framed<HomogeneousPointd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_HOMOGENEOUSPOINT_HPP
