/**
 * @file
 */

#ifndef WAVE_GEOMETRY_UNITHOMOGENEOUSPOINT_HPP
#define WAVE_GEOMETRY_UNITHOMOGENEOUSPOINT_HPP

namespace wave {

/** A spherically normalized point in P^3, with its own storage
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Quaterniond)
 *
 * The alias UnitHomogeneousPointd is provided for the typical storage type,
 * Eigen::Quaterniond.
 */
template <typename ImplType>
class UnitHomogeneousPoint
    : public HomogeneousPointBase<UnitHomogeneousPoint<ImplType>>,
      public LeafStorage<UnitHomogeneousPoint<ImplType>, ImplType> {
    static_assert(internal::is_eigen_quaternion<ImplType>::value,
                  "ImplType must be an Eigen quaternion type.");
    using Scalar = typename ImplType::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using Storage = LeafStorage<UnitHomogeneousPoint<ImplType>, ImplType>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    UnitHomogeneousPoint() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(UnitHomogeneousPoint)

    /** Construct from Eigen Matrix object */
    template <typename OtherDerived>
    UnitHomogeneousPoint(const Eigen::MatrixBase<OtherDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Construct from four scalars */
    UnitHomogeneousPoint(Scalar x, Scalar y, Scalar z, Scalar w) {
        this->storage << x, y, z, w;
    }
};

namespace internal {

template <typename ImplType>
struct traits<UnitHomogeneousPoint<ImplType>>
    : point_leaf_traits_base<UnitHomogeneousPoint<ImplType>> {
 private:
    using EigenVector3 = Eigen::Matrix<typename ImplType::Scalar, 3, 1>;

 public:
    using TangentType = Translation<EigenVector3>;
    using TangentBlocks = tmp::type_list<TangentType>;
};

}  // namespace internal

// Convenience typedefs

using UnitHomogeneousPointd = UnitHomogeneousPoint<Eigen::Quaterniond>;

/** A 3D point whose position is defined by by the vector TranslationFd<F1, F1, F2>.
 */
template <typename F1, typename F2>
using UnitHomogeneousPointFd = Framed<UnitHomogeneousPointd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_UNITHOMOGENEOUSPOINT_HPP
