/**
 * @file
 */

#ifndef WAVE_GEOMETRY_POINT_HPP
#define WAVE_GEOMETRY_POINT_HPP

namespace wave {

/** A point in R^3, with its own storage
 *
 * @tparam ImplType The type to use for storage (e.g. Eigen::Vector3d or
 * Eigen::Map<Eigen::Vector3f>)
 *
 * The alias Pointd is provided for the typical storage type, Eigen::Vector3d.
 */
template <typename ImplType>
class Point : public PointBase<Point<ImplType>>,
              public LeafStorage<Point<ImplType>, ImplType> {
    static_assert(internal::is_eigen_vector<3, ImplType>::value,
                  "ImplType must be an Eigen 3-vector type.");
    using Scalar = typename ImplType::Scalar;
    using Real = typename Eigen::NumTraits<Scalar>::Real;
    using Storage = LeafStorage<Point<ImplType>, ImplType>;

 public:
    // Inherit constructors from LeafStorage
    using Storage::Storage;
    using Storage::operator=;

    Point() = default;

    WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(Point)

    /** Construct from Eigen Matrix object */
    template <typename OtherDerived>
    Point(const Eigen::MatrixBase<OtherDerived> &m)
        : Storage{typename Storage::init_storage{}, m.derived()} {}

    /** Construct from three scalars */
    Point(Scalar x, Scalar y, Scalar z)
        : Storage{typename Storage::init_storage{}, x, y, z} {}
};

namespace internal {

template <typename ImplType>
struct traits<Point<ImplType>> : point_leaf_traits_base<Point<ImplType>> {
    using TangentType = Translation<typename ImplType::PlainObject>;
    using TangentBlocks = tmp::type_list<TangentType>;
};

}  // namespace internal

// Convenience typedefs

using Pointd = Point<Eigen::Vector3d>;

/** A 3D point whose position is defined by by the vector TranslationFd<F1, F1, F2>.
 */
template <typename F1, typename F2>
using PointFd = Framed<Pointd, F1, F2>;

}  // namespace wave

#endif  // WAVE_GEOMETRY_POINT_HPP
