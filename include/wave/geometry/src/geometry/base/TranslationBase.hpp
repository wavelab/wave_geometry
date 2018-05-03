/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_TRANSLATIONBASE_HPP
#define WAVE_GEOMETRY_TRANSLATIONBASE_HPP

namespace wave {

/** Base class for translations in R^3 */
template <typename Derived>
struct TranslationBase : public VectorBase<Derived> {
    template <typename T>
    using BaseTmpl = TranslationBase<T>;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_TRANSLATIONBASE_HPP
