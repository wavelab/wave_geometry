/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_FACTORBASE_HPP
#define WAVE_GEOMETRY_FACTORBASE_HPP

namespace wave {

/**
 * Abstract base for all factor types.
 *
 * Do not derive this class directly, instead, use a specialization of `Factor`.
 */
class FactorBase {
    using DummyArrayType = std::array<std::shared_ptr<FactorVariableBase>, 1u>;

 public:
    using const_iterator = typename DummyArrayType::const_iterator;

    virtual ~FactorBase() = default;

    /** The arity (number of variables connected) of the factor */
    virtual std::size_t size() const noexcept = 0;

    virtual const_iterator begin() const noexcept = 0;
    virtual const_iterator end() const noexcept = 0;

    /** Return true if this factor is a zero-noise prior */
    virtual bool isPerfectPrior() const noexcept = 0;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_FACTORBASE_HPP
