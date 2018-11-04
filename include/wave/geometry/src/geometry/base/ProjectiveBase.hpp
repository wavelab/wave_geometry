/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_PROJECTIVEBASE_HPP
#define WAVE_GEOMETRY_PROJECTIVEBASE_HPP

namespace wave {

/** Represents an element of projective space
 */
template <typename Derived>
struct ProjectiveBase : ExpressionBase<Derived> {
 private:
    using Scalar = internal::scalar_t<Derived>;
    using OutputType = internal::plain_output_t<Derived>;

 public:
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_PROJECTIVEBASE_HPP
