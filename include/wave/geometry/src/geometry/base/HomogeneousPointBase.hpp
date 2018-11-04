/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_HOMOGENEOUSPOINTBASE_HPP
#define WAVE_GEOMETRY_HOMOGENEOUSPOINTBASE_HPP

namespace wave {

/** Base class for homogeneous points in P^3 */
template <typename Derived>
struct HomogeneousPointBase : public ProjectiveBase<Derived> {
    template <typename T>
    using BaseTmpl = HomogeneousPointBase<T>;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_HOMOGENEOUSPOINTBASE_HPP
