/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_POINTBASE_HPP
#define WAVE_GEOMETRY_POINTBASE_HPP

namespace wave {

/** Base class for points in R^3 */
template <typename Derived>
struct PointBase : public AffineBase<Derived> {
    template <typename T>
    using BaseTmpl = PointBase<T>;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_POINTBASE_HPP
